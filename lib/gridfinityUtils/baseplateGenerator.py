import math
import adsk.core, adsk.fusion, traceback
import os

from . import const, commonUtils, filletUtils, combineUtils, faceUtils, extrudeUtils, sketchUtils, baseGenerator, patternUtils, shapeUtils, geometryUtils
from .baseGeneratorInput import BaseGeneratorInput
from .baseplateGeneratorInput import BaseplateGeneratorInput

def createGridfinityBaseplate(input: BaseplateGeneratorInput, targetComponent: adsk.fusion.Component):
    features = targetComponent.features
    cutoutInput = BaseGeneratorInput()
    cutoutInput.xyClearance = input.xyClearance
    cutoutInput.originPoint = geometryUtils.createOffsetPoint(
        targetComponent.originConstructionPoint.geometry,
        byX=-cutoutInput.xyClearance * 2,
        byY=-cutoutInput.xyClearance * 2,
    )
    cutoutInput.baseWidth = input.baseWidth + cutoutInput.xyClearance * 2
    cutoutInput.baseLength = input.baseLength + cutoutInput.xyClearance * 2
    cutoutInput.cornerFilletRadius = input.cornerFilletRadius + cutoutInput.xyClearance
    baseBody = baseGenerator.createSingleGridfinityBaseBody(cutoutInput, targetComponent)

    cuttingTools: list[adsk.fusion.BRepBody] = [baseBody]
    extraCutoutBodies: list[adsk.fusion.BRepBody] = []

    holeCenterPoint = adsk.core.Point3D.create(
        const.DIMENSION_SCREW_HOLES_OFFSET - input.xyClearance,
        const.DIMENSION_SCREW_HOLES_OFFSET - input.xyClearance,
        0
    )

    connectionHoleYTool = None
    connectionHoleXTool = None

    if input.hasSkeletonizedBottom:
        centerCutoutSketch,centerCutoutSketchCircle = baseGenerator.createCircleAtPointSketch(
            faceUtils.getBottomFace(baseBody),
            input.magnetCutoutsDiameter / 2,
            holeCenterPoint,
            targetComponent
        )
        centerCutoutSketch.name = "center bottom cutout"
        sketchUtils.convertToConstruction(centerCutoutSketch.sketchCurves)
        sketchCurves = centerCutoutSketch.sketchCurves
        dimensions = centerCutoutSketch.sketchDimensions
        constraints = centerCutoutSketch.geometricConstraints
        sketchLines = sketchCurves.sketchLines
        screwHoleCircle = sketchCurves.sketchCircles.item(0)
        arcStartingPoint = screwHoleCircle.centerSketchPoint.geometry.asVector()
        arcStartingPoint.add(adsk.core.Vector3D.create(0, max(input.magnetCutoutsDiameter, input.screwHeadCutoutDiameter) / 2 + 0.1, 0))
        arc = sketchCurves.sketchArcs.addByCenterStartSweep(
            screwHoleCircle.centerSketchPoint,
            arcStartingPoint.asPoint(),
            math.radians(90),
        )

        verticalEdgeLine = min([line for line in sketchLines if sketchUtils.isVertical(line)], key=lambda x: abs(x.startSketchPoint.geometry.x))
        horizontalEdgeLine = min([line for line in sketchLines if sketchUtils.isHorizontal(line)], key=lambda x: abs(x.startSketchPoint.geometry.y))

        baseCenterOffsetX = input.baseWidth / 2 - input.xyClearance
        baseCenterOffsetY = input.baseLength / 2 - input.xyClearance
        line1 = sketchLines.addByTwoPoints(arc.startSketchPoint, adsk.core.Point3D.create(verticalEdgeLine.startSketchPoint.geometry.x, arc.startSketchPoint.geometry.y, 0))
        line2 = sketchLines.addByTwoPoints(line1.endSketchPoint, adsk.core.Point3D.create(line1.endSketchPoint.geometry.x, baseCenterOffsetY, 0))
        line3 = sketchLines.addByTwoPoints(line2.endSketchPoint, adsk.core.Point3D.create(-baseCenterOffsetX, baseCenterOffsetY, 0))
        line4 = sketchLines.addByTwoPoints(line3.endSketchPoint, adsk.core.Point3D.create(line3.endSketchPoint.geometry.x, horizontalEdgeLine.startSketchPoint.geometry.y, 0))
        line5 = sketchLines.addByTwoPoints(line4.endSketchPoint, adsk.core.Point3D.create(arc.endSketchPoint.geometry.x, line4.endSketchPoint.geometry.y, 0))
        line6 = sketchLines.addByTwoPoints(line5.endSketchPoint, arc.endSketchPoint)
        
        constraints.addCoincident(line1.endSketchPoint, verticalEdgeLine)
        constraints.addCoincident(line6.startSketchPoint, horizontalEdgeLine)
        constraints.addCoincident(screwHoleCircle.centerSketchPoint, arc.centerSketchPoint)
        constraints.addHorizontal(line1)
        constraints.addPerpendicular(line1, line2)
        constraints.addPerpendicular(line2, line3)
        constraints.addPerpendicular(line3, line4)
        constraints.addPerpendicular(line4, line5)
        constraints.addPerpendicular(line5, line6)
        constraints.addTangent(arc, line1)
        constraints.addEqual(line1, line6)
        constraints.addEqual(line2, line5)
        dimensions.addRadialDimension(arc, arc.endSketchPoint.geometry, True)
        dimensions.addDistanceDimension(
            arc.endSketchPoint,
            line3.endSketchPoint,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            line2.endSketchPoint.geometry
            )

        centerCutoutExtrudeFeature = extrudeUtils.simpleDistanceExtrude(
            centerCutoutSketch.profiles.item(0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            input.bottomExtensionHeight,
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
            [],
            targetComponent,
        )

        constructionAxisInput: adsk.fusion.ConstructionAxisInput = targetComponent.constructionAxes.createInput()
        constructionAxisInput.setByNormalToFaceAtPoint(
            faceUtils.getBottomFace(baseBody),
            line3.endSketchPoint,
        )
        constructionAxis = targetComponent.constructionAxes.add(constructionAxisInput)
        constructionAxis.isLightBulbOn = False

        centerCutoutPattern = patternUtils.circPattern(
            commonUtils.objectCollectionFromList(centerCutoutExtrudeFeature.bodies),
            constructionAxis,
            4,
            targetComponent,
        )
        centerCutoutBody = centerCutoutExtrudeFeature.bodies.item(0)
        combineUtils.joinBodies(
            centerCutoutBody,
            commonUtils.objectCollectionFromList([body for body in list(centerCutoutPattern.bodies) if not body.name == centerCutoutBody.name]),
            targetComponent,
        )
        extraCutoutBodies.append(centerCutoutBody)
        if input.hasConnectionHoles:
            connectionHoleFaceY = min([face for face in centerCutoutBody.faces if faceUtils.isYNormal(face)], key=lambda x: x.boundingBox.minPoint.y)
            connectionHoleYTool = createConnectionHoleTool(connectionHoleFaceY, input.connectionScrewHolesDiameter / 2, input.baseWidth / 2, targetComponent)
            connectionHoleFaceX = min([face for face in centerCutoutBody.faces if faceUtils.isXNormal(face)], key=lambda x: x.boundingBox.minPoint.x)
            connectionHoleXTool = createConnectionHoleTool(connectionHoleFaceX, input.connectionScrewHolesDiameter / 2, input.baseWidth / 2, targetComponent)

    holeCuttingBodies: list[adsk.fusion.BRepBody] = []
    
    if input.hasExtendedBottom and input.hasMagnetCutouts:
        magnetSocketBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(baseBody),
            0,
            input.magnetCutoutsDepth,
            input.magnetCutoutsDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        holeCuttingBodies.append(magnetSocketBody)
    
    if input.hasExtendedBottom and input.hasScrewHoles:
        screwHoleBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(baseBody),
            0,
            input.bottomExtensionHeight,
            input.screwHolesDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        holeCuttingBodies.append(screwHoleBody)

        screwHeadHeight = const.DIMENSION_SCREW_HEAD_CUTOUT_OFFSET_HEIGHT + (input.screwHeadCutoutDiameter - input.screwHolesDiameter) / 2
        screwHeadBody = shapeUtils.simpleCylinder(
            faceUtils.getBottomFace(screwHoleBody),
            -screwHeadHeight,
            screwHeadHeight,
            input.screwHeadCutoutDiameter / 2,
            holeCenterPoint,
            targetComponent,
        )
        filletUtils.createChamfer(
            commonUtils.objectCollectionFromList(faceUtils.getTopFace(screwHeadBody).edges),
            (input.screwHeadCutoutDiameter - input.screwHolesDiameter) / 2,
            targetComponent,
        )
        holeCuttingBodies.append(screwHeadBody)

    if len(holeCuttingBodies) > 0:
        patternSpacingX = input.baseWidth - const.DIMENSION_SCREW_HOLES_OFFSET * 2
        patternSpacingY = input.baseLength - const.DIMENSION_SCREW_HOLES_OFFSET * 2
        magnetScrewCutoutsPattern = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(holeCuttingBodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (patternSpacingX, patternSpacingY),
            (2, 2),
            targetComponent
        )
        extraCutoutBodies = extraCutoutBodies + holeCuttingBodies + list(magnetScrewCutoutsPattern.bodies)

    if input.hasExtendedBottom and input.hasMagnetCutouts and input.hasGlueChannels:
        magnetRadius = input.magnetCutoutsDiameter / 2
        channelOverlap = magnetRadius * 0.3
        channelTotalRadialLength = input.glueChannelDepth + channelOverlap
        halfLen = channelTotalRadialLength / 2
        halfWidth = input.glueChannelWidth / 2

        patternSpacingX = input.baseWidth - const.DIMENSION_SCREW_HOLES_OFFSET * 2
        patternSpacingY = input.baseLength - const.DIMENSION_SCREW_HOLES_OFFSET * 2

        # 4 hole positions with direction away from center (toward outer walls)
        holeConfigs = [
            (holeCenterPoint.x, holeCenterPoint.y, -1, -1),
            (holeCenterPoint.x + patternSpacingX, holeCenterPoint.y, 1, -1),
            (holeCenterPoint.x, holeCenterPoint.y + patternSpacingY, -1, 1),
            (holeCenterPoint.x + patternSpacingX, holeCenterPoint.y + patternSpacingY, 1, 1),
        ]

        for hx, hy, dx, dy in holeConfigs:
            mag = math.sqrt(dx * dx + dy * dy)
            dirX, dirY = dx / mag, dy / mag
            perpX, perpY = -dirY, dirX

            # center of channel rectangle at the magnet circle edge
            cx = hx + magnetRadius * dirX
            cy = hy + magnetRadius * dirY

            # 4 corners of rotated rectangle
            p1x = cx - halfLen * dirX - halfWidth * perpX
            p1y = cy - halfLen * dirY - halfWidth * perpY
            p2x = cx + halfLen * dirX - halfWidth * perpX
            p2y = cy + halfLen * dirY - halfWidth * perpY
            p3x = cx + halfLen * dirX + halfWidth * perpX
            p3y = cy + halfLen * dirY + halfWidth * perpY
            p4x = cx - halfLen * dirX + halfWidth * perpX
            p4y = cy - halfLen * dirY + halfWidth * perpY

            channelPlaneInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
            channelPlaneInput.setByOffset(targetComponent.xYConstructionPlane, adsk.core.ValueInput.createByReal(-const.BIN_BASE_HEIGHT))
            channelPlane = targetComponent.constructionPlanes.add(channelPlaneInput)
            channelPlane.isLightBulbOn = False

            channelSketch: adsk.fusion.Sketch = targetComponent.sketches.add(channelPlane)
            channelSketch.name = "Glue escape channel"

            sp1 = channelSketch.modelToSketchSpace(adsk.core.Point3D.create(p1x, p1y, 0))
            sp2 = channelSketch.modelToSketchSpace(adsk.core.Point3D.create(p2x, p2y, 0))
            sp3 = channelSketch.modelToSketchSpace(adsk.core.Point3D.create(p3x, p3y, 0))
            sp4 = channelSketch.modelToSketchSpace(adsk.core.Point3D.create(p4x, p4y, 0))
            sp1.z = 0; sp2.z = 0; sp3.z = 0; sp4.z = 0

            lines = channelSketch.sketchCurves.sketchLines
            lines.addByTwoPoints(sp1, sp2)
            lines.addByTwoPoints(sp2, sp3)
            lines.addByTwoPoints(sp3, sp4)
            lines.addByTwoPoints(sp4, sp1)

            channelExtrude = extrudeUtils.simpleDistanceExtrude(
                channelSketch.profiles.item(0),
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                input.magnetCutoutsDepth,
                adsk.fusion.ExtentDirections.NegativeExtentDirection,
                [],
                targetComponent,
            )
            channelExtrude.name = "Glue escape channel"
            extraCutoutBodies.append(channelExtrude.bodies.item(0))

            # semicircle cap at the outer end of the rectangle
            capCenterPoint = adsk.core.Point3D.create(
                cx + halfLen * dirX,
                cy + halfLen * dirY,
                0,
            )
            capBody = shapeUtils.simpleCylinder(
                faceUtils.getBottomFace(baseBody),
                0,
                input.magnetCutoutsDepth,
                halfWidth,
                capCenterPoint,
                targetComponent,
            )
            capBody.name = "Glue channel cap"
            extraCutoutBodies.append(capBody)

    if len(extraCutoutBodies) > 0:
        combineUtils.joinBodies(
            baseBody,
            commonUtils.objectCollectionFromList(extraCutoutBodies),
            targetComponent,
        )
    
    # replicate base in rectangular pattern
    rectangularPatternFeatures: adsk.fusion.RectangularPatternFeatures = features.rectangularPatternFeatures
    patternInputBodies = adsk.core.ObjectCollection.create()
    patternInputBodies.add(baseBody)
    patternInput = rectangularPatternFeatures.createInput(patternInputBodies,
        targetComponent.xConstructionAxis,
        adsk.core.ValueInput.createByReal(input.baseplateWidth),
        adsk.core.ValueInput.createByReal(input.baseWidth),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    patternInput.directionTwoEntity = targetComponent.yConstructionAxis
    patternInput.quantityTwo = adsk.core.ValueInput.createByReal(input.baseplateLength)
    patternInput.distanceTwo = adsk.core.ValueInput.createByReal(input.baseLength)
    rectangularPattern = rectangularPatternFeatures.add(patternInput)
    cuttingTools = cuttingTools + list(rectangularPattern.bodies)

    # create baseplate body
    baseplateTrueWidth = input.baseplateWidth * input.baseWidth - input.xyClearance * 2
    baseplateTrueLength = input.baseplateLength * input.baseLength - input.xyClearance * 2
    binInterfaceBody = shapeUtils.simpleBox(
        targetComponent.xYConstructionPlane,
        0,
        input.baseplateWidth * input.baseWidth - input.xyClearance * 2,
        input.baseplateLength * input.baseLength - input.xyClearance * 2,
        -const.BIN_BASE_HEIGHT,
        targetComponent.originConstructionPoint.geometry,
        targetComponent,
    )

    if input.binZClearance > 0:
        binZClearance = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                baseplateTrueWidth + input.paddingLeft + input.paddingRight,
                baseplateTrueLength + input.paddingBottom + input.paddingTop,
                -input.binZClearance,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
        binZClearance.name = "Top negative volume"
        cuttingTools.append(binZClearance)

    if input.hasPadding:
        paddingHeigth = const.BIN_BASE_HEIGHT
        mergeTools = []
        if input.paddingLeft > 0:
            paddingLeftBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                input.paddingLeft,
                baseplateTrueLength + input.paddingBottom + input.paddingTop,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingLeftBody.name = "Padding left"
            mergeTools.append(paddingLeftBody)
        if input.paddingTop > 0:
            paddingTopBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                baseplateTrueWidth + input.paddingLeft + input.paddingRight,
                input.paddingTop,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=baseplateTrueLength
                ),
                targetComponent
            )
            paddingTopBody.name = "Padding top"
            mergeTools.append(paddingTopBody)
        if input.paddingRight > 0:
            paddingRightBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                input.paddingRight,
                baseplateTrueLength + input.paddingTop + input.paddingBottom,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=baseplateTrueWidth,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingRightBody.name = "Padding right"
            mergeTools.append(paddingRightBody)
        if input.paddingBottom > 0:
            paddingBottomBody = shapeUtils.simpleBox(
                targetComponent.xYConstructionPlane,
                0,
                baseplateTrueWidth + input.paddingLeft + input.paddingRight,
                input.paddingBottom,
                -paddingHeigth,
                geometryUtils.createOffsetPoint(
                    targetComponent.originConstructionPoint.geometry,
                    byX=-input.paddingLeft,
                    byY=-input.paddingBottom
                ),
                targetComponent
            )
            paddingBottomBody.name = "Padding bottom"
            mergeTools.append(paddingBottomBody)
        if len(mergeTools) > 0:
            paddingCombineFeature = combineUtils.joinBodies(
                binInterfaceBody,
                commonUtils.objectCollectionFromList(mergeTools),
                targetComponent,
            )
            paddingCombineFeature.name = "Combine base with padding bodies"
            binInterfaceBody = paddingCombineFeature.bodies.item(0)

    cornerFillet = filletUtils.filletEdgesByLength(
        binInterfaceBody.faces,
        input.cornerFilletRadius - input.xyClearance,
        const.BIN_BASE_HEIGHT,
        targetComponent,
        )
    cornerFillet.name = "Round outer corners"
    
    if input.hasExtendedBottom:
        baseplateBottomLayer = extrudeUtils.simpleDistanceExtrude(
            faceUtils.getBottomFace(binInterfaceBody),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            input.bottomExtensionHeight,
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
            [],
            targetComponent,
        )
        baseplateBottomLayerBody = baseplateBottomLayer.bodies.item(0)
        combineUtils.joinBodies(binInterfaceBody, commonUtils.objectCollectionFromList([baseplateBottomLayerBody]), targetComponent)

    # Locking tabs — shared setup
    hasAnyTab = (input.tabLeftType != const.TAB_TYPE_NONE
        or input.tabRightType != const.TAB_TYPE_NONE
        or input.tabTopType != const.TAB_TYPE_NONE
        or input.tabBottomType != const.TAB_TYPE_NONE)

    if hasAnyTab and input.hasExtendedBottom:
        tabHeight = const.DIMENSION_TAB_HEIGHT
        tabBaseWidth = const.DIMENSION_TAB_BASE_WIDTH
        tabAngleRad = math.radians(const.DIMENSION_TAB_ANGLE_DEG)
        tabTipWidth = tabBaseWidth + 2.0 * tabHeight / math.tan(tabAngleRad)
        cl = input.tabClearance

        edgeLeft = -(input.paddingLeft if input.hasPadding else 0)
        edgeRight = baseplateTrueWidth + (input.paddingRight if input.hasPadding else 0)
        edgeBottom = -(input.paddingBottom if input.hasPadding else 0)
        edgeTop = baseplateTrueLength + (input.paddingTop if input.hasPadding else 0)

        def _createTabBody(cx, cy, bw, tw, h, axis, sign, targetComponent, applyFillet=True):
            tabZHeight = const.DIMENSION_TAB_HEIGHT
            filletRadius = 0.05

            extBottomTop = -const.BIN_BASE_HEIGHT
            extBottomBot = -(const.BIN_BASE_HEIGHT + input.bottomExtensionHeight)
            tabTopZ = (extBottomTop + extBottomBot) / 2 + tabZHeight / 2

            planeInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
            planeInput.setByOffset(targetComponent.xYConstructionPlane, adsk.core.ValueInput.createByReal(tabTopZ))
            plane = targetComponent.constructionPlanes.add(planeInput)
            plane.isLightBulbOn = False
            sketch: adsk.fusion.Sketch = targetComponent.sketches.add(plane)
            sketch.name = "Locking tab"

            if axis == 'x':
                pts = [
                    adsk.core.Point3D.create(cx - bw / 2, cy, 0),
                    adsk.core.Point3D.create(cx + bw / 2, cy, 0),
                    adsk.core.Point3D.create(cx + tw / 2, cy + sign * h, 0),
                    adsk.core.Point3D.create(cx - tw / 2, cy + sign * h, 0),
                ]
            else:
                pts = [
                    adsk.core.Point3D.create(cx, cy - bw / 2, 0),
                    adsk.core.Point3D.create(cx, cy + bw / 2, 0),
                    adsk.core.Point3D.create(cx + sign * h, cy + tw / 2, 0),
                    adsk.core.Point3D.create(cx + sign * h, cy - tw / 2, 0),
                ]

            spts = []
            for p in pts:
                sp = sketch.modelToSketchSpace(p)
                sp.z = 0
                spts.append(sp)

            lines = sketch.sketchCurves.sketchLines
            lines.addByTwoPoints(spts[0], spts[1])
            lines.addByTwoPoints(spts[1], spts[2])
            lines.addByTwoPoints(spts[2], spts[3])
            lines.addByTwoPoints(spts[3], spts[0])

            ext = extrudeUtils.simpleDistanceExtrude(
                sketch.profiles.item(0),
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                tabZHeight,
                adsk.fusion.ExtentDirections.NegativeExtentDirection,
                [],
                targetComponent,
            )
            ext.name = "Locking tab"
            body = ext.bodies.item(0)

            if applyFillet:
                try:
                    tol = 0.001
                    minZ = min(v.geometry.z for v in body.vertices)
                    edges = adsk.core.ObjectCollection.create()
                    for edge in body.edges:
                        sv = edge.startVertex.geometry
                        ev = edge.endVertex.geometry
                        # Skip mating face edges
                        if axis == 'x':
                            onMatingFace = abs(sv.y - cy) < tol and abs(ev.y - cy) < tol
                        else:
                            onMatingFace = abs(sv.x - cx) < tol and abs(ev.x - cx) < tol
                        # Skip bottom face edges
                        onBottomFace = abs(sv.z - minZ) < tol and abs(ev.z - minZ) < tol
                        if not onMatingFace and not onBottomFace:
                            edges.add(edge)
                    if edges.count > 0:
                        filletFeatures = targetComponent.features.filletFeatures
                        filletInput = filletFeatures.createInput()
                        filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), True)
                        filletFeatures.add(filletInput)
                except:
                    pass
            return body

        def _createTabsForEdge(tabType, edgeCoord, axis, numUnits, unitSize, onlyType=None):
            if tabType == const.TAB_TYPE_NONE:
                return
            if onlyType is not None and tabType != onlyType:
                return
            isMale = tabType == const.TAB_TYPE_MALE
            tabBodies = []
            for u in range(int(numUnits)):
                for i in range(2):
                    centerAlongEdge = u * unitSize + (2 * i + 1) * unitSize / 4 - input.xyClearance
                    if isMale:
                        bw, tw, h = tabBaseWidth, tabTipWidth, tabHeight
                        if axis == 'x':
                            sign = -1 if edgeCoord <= baseplateTrueLength / 2 else 1
                        else:
                            sign = -1 if edgeCoord <= baseplateTrueWidth / 2 else 1
                    else:
                        # Female: narrow opening at surface, widens inward (matches male tab)
                        bw = tabBaseWidth + 2 * cl
                        tw = tabTipWidth + 2 * cl
                        h = tabHeight + cl
                        if axis == 'x':
                            sign = 1 if edgeCoord <= baseplateTrueLength / 2 else -1
                        else:
                            sign = 1 if edgeCoord <= baseplateTrueWidth / 2 else -1
                    if axis == 'x':
                        body = _createTabBody(centerAlongEdge, edgeCoord, bw, tw, h, axis, sign, targetComponent, applyFillet=True)
                    else:
                        body = _createTabBody(edgeCoord, centerAlongEdge, bw, tw, h, axis, sign, targetComponent, applyFillet=True)
                    tabBodies.append(body)
            if len(tabBodies) > 0:
                bodyCollection = commonUtils.objectCollectionFromList(tabBodies)
                if isMale:
                    combineUtils.joinBodies(binInterfaceBody, bodyCollection, targetComponent)
                else:
                    combineUtils.cutBody(binInterfaceBody, bodyCollection, targetComponent)

        # Create MALE tabs before chamfer so chamfer blends with tab edges
        _createTabsForEdge(input.tabBottomType, edgeBottom, 'x', input.baseplateWidth, input.baseWidth, onlyType=const.TAB_TYPE_MALE)
        _createTabsForEdge(input.tabTopType, edgeTop, 'x', input.baseplateWidth, input.baseWidth, onlyType=const.TAB_TYPE_MALE)
        _createTabsForEdge(input.tabLeftType, edgeLeft, 'y', input.baseplateLength, input.baseLength, onlyType=const.TAB_TYPE_MALE)
        _createTabsForEdge(input.tabRightType, edgeRight, 'y', input.baseplateLength, input.baseLength, onlyType=const.TAB_TYPE_MALE)

        # Create FEMALE pockets before chamfer too so chamfer blends with pocket edges
        _createTabsForEdge(input.tabBottomType, edgeBottom, 'x', input.baseplateWidth, input.baseWidth, onlyType=const.TAB_TYPE_FEMALE)
        _createTabsForEdge(input.tabTopType, edgeTop, 'x', input.baseplateWidth, input.baseWidth, onlyType=const.TAB_TYPE_FEMALE)
        _createTabsForEdge(input.tabLeftType, edgeLeft, 'y', input.baseplateLength, input.baseLength, onlyType=const.TAB_TYPE_FEMALE)
        _createTabsForEdge(input.tabRightType, edgeRight, 'y', input.baseplateLength, input.baseLength, onlyType=const.TAB_TYPE_FEMALE)

    bottomFace = faceUtils.getBottomFace(binInterfaceBody)
    allBottomEdges = adsk.core.ObjectCollection.create()
    for edge in bottomFace.edges:
        allBottomEdges.add(edge)
    bottomChamfer = filletUtils.createChamfer(allBottomEdges, 0.05, targetComponent)
    bottomChamfer.name = "Bottom chamfer"

    if not connectionHoleYTool is None and not connectionHoleXTool is None:
        holeToolsXFeature = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(connectionHoleXTool.bodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (input.baseWidth, input.baseLength),
            (1, input.baseplateLength),
            targetComponent
        )
        connectionHoleXToolList = list(connectionHoleXTool.bodies) + list(holeToolsXFeature.bodies)

        holeToolsYFeature = patternUtils.recPattern(
            commonUtils.objectCollectionFromList(connectionHoleYTool.bodies),
            (targetComponent.xConstructionAxis, targetComponent.yConstructionAxis),
            (input.baseLength, input.baseLength),
            (input.baseplateWidth, 1),
            targetComponent
        )
        connectionHoleYToolList = list(connectionHoleYTool.bodies) + list(holeToolsYFeature.bodies)

        constructionPlaneXZInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
        constructionPlaneXZInput.setByOffset(targetComponent.xZConstructionPlane, adsk.core.ValueInput.createByReal(input.baseplateLength * input.baseLength / 2 - input.xyClearance))
        constructionPlaneXZ = targetComponent.constructionPlanes.add(constructionPlaneXZInput)
        constructionPlaneXZ.isLightBulbOn = False

        constructionPlaneYZInput: adsk.fusion.ConstructionPlaneInput = targetComponent.constructionPlanes.createInput()
        constructionPlaneYZInput.setByOffset(targetComponent.yZConstructionPlane, adsk.core.ValueInput.createByReal(input.baseplateWidth * input.baseWidth / 2 - input.xyClearance))
        constructionPlaneYZ = targetComponent.constructionPlanes.add(constructionPlaneYZInput)
        constructionPlaneYZ.isLightBulbOn = False

        mirrorConnectionHolesYZInput = features.mirrorFeatures.createInput(commonUtils.objectCollectionFromList(connectionHoleXToolList), constructionPlaneYZ)
        mirrorConnectionHolesYZ = features.mirrorFeatures.add(mirrorConnectionHolesYZInput)

        mirrorConnectionHolesXZInput = features.mirrorFeatures.createInput(commonUtils.objectCollectionFromList(connectionHoleYToolList), constructionPlaneXZ)
        mirrorConnectionHolesXZ = features.mirrorFeatures.add(mirrorConnectionHolesXZInput)

        cuttingTools = cuttingTools + list(mirrorConnectionHolesYZ.bodies) + list(mirrorConnectionHolesXZ.bodies) + connectionHoleYToolList + connectionHoleXToolList


    # cut everything
    toolBodies = commonUtils.objectCollectionFromList(cuttingTools)
    finalCut = combineUtils.cutBody(
        binInterfaceBody,
        toolBodies,
        targetComponent,
    )
    finalCut.name = "Final baseplate cut"

    return binInterfaceBody

def createConnectionHoleTool(connectionHoleFace: adsk.fusion.BRepFace, diameter: float, depth: float, targetComponent: adsk.fusion.Component):
    connectionHoleSketch: adsk.fusion.Sketch = targetComponent.sketches.add(connectionHoleFace)
    connectionHoleSketch.name = "side connector hole"
    sketchCurves = connectionHoleSketch.sketchCurves
    dimensions = connectionHoleSketch.sketchDimensions
    constraints = connectionHoleSketch.geometricConstraints
    sketchUtils.convertToConstruction(sketchCurves)
    [sketchHorizontalEdge1, sketchHorizontalEdge2] = [line for line in sketchCurves.sketchLines if sketchUtils.isHorizontal(line)]
    line1 = sketchCurves.sketchLines.addByTwoPoints(sketchHorizontalEdge1.startSketchPoint.geometry, sketchHorizontalEdge2.endSketchPoint.geometry)
    line1.isConstruction = True
    constraints.addMidPoint(line1.startSketchPoint, sketchHorizontalEdge1)
    constraints.addMidPoint(line1.endSketchPoint, sketchHorizontalEdge2)
    
    circle = sketchCurves.sketchCircles.addByCenterRadius(
        connectionHoleSketch.originPoint.geometry,
        diameter
    )
    constraints.addMidPoint(circle.centerSketchPoint, line1)
    dimensions.addRadialDimension(circle, line1.startSketchPoint.geometry, True)
    connectionHoleTool = extrudeUtils.simpleDistanceExtrude(
        connectionHoleSketch.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        depth,
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
        [],
        targetComponent,
    )
    return connectionHoleTool