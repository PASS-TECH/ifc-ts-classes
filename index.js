"use strict";
/***********
generated template classes for ./IFC4.xsd 22/06/2022, 15:45:10
***********/
var __extends = (this && this.__extends) || (function () {
    var extendStatics = function (d, b) {
        extendStatics = Object.setPrototypeOf ||
            ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
            function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
        return extendStatics(d, b);
    };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
var _a, _b, _c, _d, _e, _f, _g, _h;
Object.defineProperty(exports, "__esModule", { value: true });
exports.IfcStructuralActivity = exports.IfcSpatialElement = exports.IfcGrid = exports.IfcElement = exports.IfcTypeProduct = exports.IfcProduct = exports.IfcActor = exports.IfcTypeObject = exports.IfcRevolvedAreaSolidTapered = exports.IfcRelSpaceBoundary2ndLevel = exports.IfcObject = exports.IfcLightSourceSpot = exports.IfcExtrudedAreaSolidTapered = exports.IfcContext = exports.IfcArbitraryProfileDefWithVoids = exports.IfcTextureMap = exports.IfcTextLiteralWithExtent = exports.IfcSurfaceStyleRendering = exports.IfcSurfaceOfRevolution = exports.IfcSurfaceOfLinearExtrusion = exports.IfcSurfaceCurveSweptAreaSolid = exports.IfcSubedge = exports.IfcStyledItem = exports.IfcStructuralPointConnection = exports.IfcStructuralCurveConnection = exports.IfcRevolvedAreaSolid = exports.IfcRelSpaceBoundary1stLevel = exports.IfcRelConnectsWithRealizingElements = exports.IfcRelConnectsWithEccentricity = exports.IfcRelAssociatesMaterial = exports.IfcRelAssociatesLibrary = exports.IfcRelAssociatesDocument = exports.IfcRelAssociatesConstraint = exports.IfcRelAssociatesClassification = exports.IfcRelAssociatesApproval = exports.IfcRelAssignsToResource = exports.IfcRelAssignsToProduct = exports.IfcRelAssignsToProcess = exports.IfcRelAssignsToGroup = exports.IfcRelAssignsToControl = exports.IfcRelAssignsToActor = exports.IfcRegularTimeSeries = exports.IfcProductDefinitionShape = exports.IfcPresentationLayerWithStyle = exports.IfcPolygonalFaceSet = exports.IfcPolygonalBoundedHalfSpace = exports.IfcParameterizedProfileDef = exports.IfcObjective = exports.IfcObjectDefinition = exports.IfcMetric = exports.IfcMaterialProfileSetUsageTapering = exports.IfcMaterialProfileSet = exports.IfcMaterialProfile = exports.IfcMaterialLayerSet = exports.IfcMaterialLayer = exports.IfcMaterialConstituentSet = exports.IfcMaterialConstituent = exports.IfcMaterial = exports.IfcMappedItem = exports.IfcLightSourcePositional = exports.IfcLightSourceGoniometric = exports.IfcLightSourceDirectional = exports.IfcIrregularTimeSeries = exports.IfcIndexedTextureMap = exports.IfcFixedReferenceSweptAreaSolid = exports.IfcFaceSurface = exports.IfcExtrudedAreaSolid = exports.IfcEdgeCurve = exports.IfcDerivedProfileDef = exports.IfcConversionBasedUnit = exports.IfcCartesianTransformationOperator3D = exports.IfcBoxedHalfSpace = exports.IfcBoundaryNodeConditionWarping = exports.IfcBlobTexture = exports.IfcAxis2Placement3D = exports.IfcAxis2Placement2D = exports.IfcAxis1Placement = exports.IfcArbitraryOpenProfileDef = exports.IfcArbitraryClosedProfileDef = exports.IfcWorkTime = exports.IfcWorkControl = exports.IfcWorkCalendar = exports.IfcWindowPanelProperties = exports.IfcWindowLiningProperties = exports.IfcVirtualGridIntersection = exports.IfcVertexPoint = exports.IfcVertexLoop = exports.IfcVector = exports.IfcUnitAssignment = exports.IfcTrimmedCurve = exports.IfcTimeSeriesValue = exports.IfcTimeSeries = exports.IfcTextureCoordinate = exports.IfcTextStyleTextModel = exports.IfcTextStyleForDefinedFont = exports.IfcTextStyleFontModel = exports.IfcTextStyle = exports.IfcTextLiteral = exports.IfcTessellatedFaceSet = exports.IfcTaskTimeRecurring = exports.IfcTask = exports.IfcTableRow = exports.IfcTableColumn = exports.IfcTable = exports.IfcSweptSurface = exports.IfcSweptDiskSolid = exports.IfcSweptAreaSolid = exports.IfcSurfaceTexture = exports.IfcSurfaceStyleWithTextures = exports.IfcSurfaceStyleShading = exports.IfcSurfaceStyleLighting = exports.IfcSurfaceStyle = exports.IfcSurfaceCurve = exports.IfcStructuralResultGroup = exports.IfcStructuralLoadConfiguration = exports.IfcStructuralCurveMember = exports.IfcStructuralConnection = exports.IfcStructuralAnalysisModel = exports.IfcSite = exports.IfcSimplePropertyTemplate = exports.IfcShellBasedSurfaceModel = exports.IfcSectionedSpine = exports.IfcSectionReinforcementProperties = exports.IfcSectionProperties = exports.IfcRoot = exports.IfcResourceConstraintRelationship = exports.IfcResourceApprovalRelationship = exports.IfcRepresentationMap = exports.IfcRepresentationItem = exports.IfcRepresentation = exports.IfcRelVoidsElement = exports.IfcRelSpaceBoundary = exports.IfcRelServicesBuildings = exports.IfcRelSequence = exports.IfcRelReferencedInSpatialStructure = exports.IfcRelProjectsElement = exports.IfcRelNests = exports.IfcRelInterferesElements = exports.IfcRelFlowControlElements = exports.IfcRelFillsElement = exports.IfcRelDefinesByType = exports.IfcRelDefinesByTemplate = exports.IfcRelDefinesByProperties = exports.IfcRelDeclares = exports.IfcRelCoversSpaces = exports.IfcRelCoversBldgElements = exports.IfcRelContainedInSpatialStructure = exports.IfcRelConnectsStructuralMember = exports.IfcRelConnectsStructuralActivity = exports.IfcRelConnectsPorts = exports.IfcRelConnectsPortToElement = exports.IfcRelConnectsElements = exports.IfcRelAssociates = exports.IfcRelAssigns = exports.IfcRelAggregates = exports.IfcReinforcingMeshType = exports.IfcReinforcingBarType = exports.IfcReinforcementDefinitionProperties = exports.IfcReference = exports.IfcRectangularTrimmedSurface = exports.IfcRationalBSplineSurfaceWithKnots = exports.IfcPropertyTableValue = exports.IfcPropertySingleValue = exports.IfcPropertySet = exports.IfcPropertyReferenceValue = exports.IfcPropertyListValue = exports.IfcPropertyEnumeration = exports.IfcPropertyEnumeratedValue = exports.IfcPropertyDependencyRelationship = exports.IfcPropertyBoundedValue = exports.IfcProjectedCRS = exports.IfcPresentationStyleAssignment = exports.IfcPresentationLayerAssignment = exports.IfcPolyline = exports.IfcPolyLoop = exports.IfcPointOnSurface = exports.IfcPointOnCurve = exports.IfcPlanarBox = exports.IfcPlacement = exports.IfcPhysicalSimpleQuantity = exports.IfcPhysicalComplexQuantity = exports.IfcPersonAndOrganization = exports.IfcPerson = exports.IfcPermeableCoveringProperties = exports.IfcPcurve = exports.IfcPath = exports.IfcOwnerHistory = exports.IfcOrganizationRelationship = exports.IfcOrganization = exports.IfcOpeningElement = exports.IfcOffsetCurve3D = exports.IfcOffsetCurve2D = exports.IfcNamedUnit = exports.IfcMeasureWithUnit = exports.IfcMaterialRelationship = exports.IfcMaterialProfileSetUsage = exports.IfcMaterialList = exports.IfcMaterialLayerSetUsage = exports.IfcMaterialDefinition = exports.IfcMaterialClassificationRelationship = exports.IfcManifoldSolidBrep = exports.IfcLocalPlacement = exports.IfcLine = exports.IfcLightSource = exports.IfcLightIntensityDistribution = exports.IfcLibraryReference = exports.IfcLibraryInformation = exports.IfcLagTime = exports.IfcIrregularTimeSeriesValue = exports.IfcInventory = exports.IfcIndexedPolyCurve = exports.IfcIndexedColourMap = exports.IfcHalfSpaceSolid = exports.IfcGridPlacement = exports.IfcGridAxis = exports.IfcGeometricSet = exports.IfcGeometricRepresentationContext = exports.IfcFillAreaStyleTiles = exports.IfcFillAreaStyleHatching = exports.IfcFillAreaStyle = exports.IfcFacetedBrepWithVoids = exports.IfcFaceBound = exports.IfcFaceBasedSurfaceModel = exports.IfcExternalReferenceRelationship = exports.IfcExtendedProperties = exports.IfcEvent = exports.IfcElementarySurface = exports.IfcElementQuantity = exports.IfcEdgeLoop = exports.IfcEdge = exports.IfcDoorPanelProperties = exports.IfcDoorLiningProperties = exports.IfcDocumentReference = exports.IfcDocumentInformationRelationship = exports.IfcDocumentInformation = exports.IfcDerivedUnitElement = exports.IfcDerivedUnit = exports.IfcCurveStyleFontAndScaling = exports.IfcCurveStyleFont = exports.IfcCurveStyle = exports.IfcCurveBoundedSurface = exports.IfcCurveBoundedPlane = exports.IfcCurrencyRelationship = exports.IfcCsgSolid = exports.IfcCsgPrimitive3D = exports.IfcCostItem = exports.IfcCoordinateOperation = exports.IfcConstructionResourceType = exports.IfcConstructionResource = exports.IfcConstraint = exports.IfcConnectionVolumeGeometry = exports.IfcConnectionSurfaceGeometry = exports.IfcConnectionPointGeometry = exports.IfcConnectionCurveGeometry = exports.IfcConnectedFaceSet = exports.IfcConic = exports.IfcCompositeCurveSegment = exports.IfcComplexPropertyTemplate = exports.IfcComplexProperty = exports.IfcClassificationReference = exports.IfcClassification = exports.IfcCartesianTransformationOperator = exports.IfcBuilding = exports.IfcBoundingBox = exports.IfcBoundaryNodeCondition = exports.IfcBoundaryFaceCondition = exports.IfcBoundaryEdgeCondition = exports.IfcBooleanResult = exports.IfcBSplineSurface = exports.IfcBSplineCurve = exports.IfcAsset = exports.IfcApprovalRelationship = exports.IfcApproval = exports.IfcAppliedValue = exports.IfcApplication = exports.IfcAnnotationFillArea = exports.IfcAdvancedBrepWithVoids = exports.IfcWarpingMomentMeasure = exports.IfcWarpingConstantMeasure = exports.IfcVolumetricFlowRateMeasure = exports.IfcVolumeMeasure = exports.IfcVaporPermeabilityMeasure = exports.IfcTorqueMeasure = exports.IfcTimeStamp = exports.IfcTimeMeasure = exports.IfcTime = exports.IfcThermodynamicTemperatureMeasure = exports.IfcThermalTransmittanceMeasure = exports.IfcThermalResistanceMeasure = exports.IfcThermalExpansionCoefficientMeasure = exports.IfcThermalConductivityMeasure = exports.IfcThermalAdmittanceMeasure = exports.IfcTextFontName = exports.IfcText = exports.IfcTemperatureRateOfChangeMeasure = exports.IfcTemperatureGradientMeasure = exports.IfcSpecularRoughness = exports.IfcSpecularExponent = exports.IfcSpecificHeatCapacityMeasure = exports.IfcSoundPressureMeasure = exports.IfcSoundPressureLevelMeasure = exports.IfcSoundPowerMeasure = exports.IfcSoundPowerLevelMeasure = exports.IfcSolidAngleMeasure = exports.IfcShearModulusMeasure = exports.IfcSectionalAreaIntegralMeasure = exports.IfcSectionModulusMeasure = exports.IfcRotationalStiffnessMeasure = exports.IfcRotationalMassMeasure = exports.IfcRotationalFrequencyMeasure = exports.IfcReal = exports.IfcRatioMeasure = exports.IfcRadioActivityMeasure = exports.IfcPressureMeasure = exports.IfcPowerMeasure = exports.IfcPositiveRatioMeasure = exports.IfcPositivePlaneAngleMeasure = exports.IfcPositiveLengthMeasure = exports.IfcPositiveInteger = exports.IfcPlaneAngleMeasure = exports.IfcPlanarForceMeasure = exports.IfcParameterValue = exports.IfcPHMeasure = exports.IfcNumericMeasure = exports.IfcNullStyle = exports.IfcNormalisedRatioMeasure = exports.IfcNonNegativeLengthMeasure = exports.IfcMonetaryMeasure = exports.IfcMomentOfInertiaMeasure = exports.IfcMolecularWeightMeasure = exports.IfcMoistureDiffusivityMeasure = exports.IfcModulusOfSubgradeReactionMeasure = exports.IfcModulusOfRotationalSubgradeReactionMeasure = exports.IfcModulusOfLinearSubgradeReactionMeasure = exports.IfcModulusOfElasticityMeasure = exports.IfcMassPerLengthMeasure = exports.IfcMassMeasure = exports.IfcMassFlowRateMeasure = exports.IfcMassDensityMeasure = exports.IfcMagneticFluxMeasure = exports.IfcMagneticFluxDensityMeasure = exports.IfcLuminousIntensityMeasure = exports.IfcLuminousIntensityDistributionMeasure = exports.IfcLuminousFluxMeasure = exports.IfcLogical = exports.IfcLinearVelocityMeasure = exports.IfcLinearStiffnessMeasure = exports.IfcLinearMomentMeasure = exports.IfcLinearForceMeasure = exports.IfcLengthMeasure = exports.IfcLabel = exports.IfcKinematicViscosityMeasure = exports.IfcIsothermalMoistureCapacityMeasure = exports.IfcIonConcentrationMeasure = exports.IfcIntegerCountRateMeasure = exports.IfcInteger = exports.IfcInductanceMeasure = exports.IfcIlluminanceMeasure = exports.IfcIdentifier = exports.IfcHeatingValueMeasure = exports.IfcHeatFluxDensityMeasure = exports.IfcFrequencyMeasure = exports.IfcForceMeasure = exports.IfcEnergyMeasure = exports.IfcElectricVoltageMeasure = exports.IfcElectricResistanceMeasure = exports.IfcElectricCurrentMeasure = exports.IfcElectricConductanceMeasure = exports.IfcElectricChargeMeasure = exports.IfcElectricCapacitanceMeasure = exports.IfcDynamicViscosityMeasure = exports.IfcDuration = exports.IfcDoseEquivalentMeasure = exports.IfcDescriptiveMeasure = exports.IfcDateTime = exports.IfcDate = exports.IfcCurvatureMeasure = exports.IfcCountMeasure = exports.IfcContextDependentMeasure = exports.IfcBoolean = exports.IfcAreaMeasure = exports.IfcAreaDensityMeasure = exports.IfcAngularVelocityMeasure = exports.IfcAmountOfSubstanceMeasure = exports.IfcAccelerationMeasure = exports.IfcAbsorbedDoseMeasure = exports.HexBinary = exports.Entity = exports.IfcPropertySetDefinitionSet = exports.IfcLineIndex = exports.IfcCompoundPlaneAngleMeasure = exports.IfcComplexNumber = exports.IfcBinary = exports.IfcArcIndex = exports.IfcZone = exports.IfcZShapeProfileDef = exports.IfcWorkSchedule = exports.IfcWorkPlan = exports.IfcWindowType = exports.IfcWindowStyle = exports.IfcWindowStandardCase = exports.IfcWindow = exports.IfcWasteTerminalType = exports.IfcWasteTerminal = exports.IfcWallType = exports.IfcWallStandardCase = exports.IfcWallElementedCase = exports.IfcWall = exports.IfcVoidingFeature = exports.IntersectingAxes = exports.IfcVirtualElement = exports.IfcVibrationIsolatorType = exports.IfcVibrationIsolator = exports.IfcVertex = exports.IfcValveType = exports.IfcValve = exports.IfcUnitaryEquipmentType = exports.IfcUnitaryEquipment = exports.IfcUnitaryControlElementType = exports.IfcUnitaryControlElement = exports.IfcUShapeProfileDef = exports.IfcTypeResource = exports.IfcTypeProcess = exports.IfcTubeBundleType = exports.IfcTubeBundle = exports.IfcTriangulatedFaceSet = exports.IfcTrapeziumProfileDef = exports.IfcTransportElementType = exports.IfcTransportElement = exports.IfcTransformerType = exports.IfcTransformer = exports.IfcToroidalSurface = exports.IfcTopologyRepresentation = exports.IfcTopologicalRepresentationItem = exports.IfcTimePeriod = exports.IfcTextureVertexList = exports.IfcTextureVertex = exports.IfcTextureCoordinateGenerator = exports.IfcTessellatedItem = exports.IfcTendonType = exports.IfcTendonAnchorType = exports.IfcTendonAnchor = exports.IfcTendon = exports.IfcTelecomAddress = exports.IfcTaskType = exports.IfcTaskTime = exports.IfcTankType = exports.IfcTank = exports.IfcTShapeProfileDef = exports.IfcSystemFurnitureElementType = exports.IfcSystemFurnitureElement = exports.IfcSystem = exports.IfcSwitchingDeviceType = exports.IfcSwitchingDevice = exports.IfcSweptDiskSolidPolygonal = exports.IfcSurfaceStyleRefraction = exports.IfcSurfaceReinforcementArea = exports.IfcSurfaceFeature = exports.AssociatedGeometry = exports.IfcSurface = exports.IfcSubContractResourceType = exports.IfcSubContractResource = exports.IfcStyledRepresentation = exports.IfcStyleModel = exports.IfcStructuralSurfaceReaction = exports.IfcStructuralSurfaceMemberVarying = exports.IfcStructuralSurfaceMember = exports.IfcStructuralSurfaceConnection = exports.IfcStructuralSurfaceAction = exports.IfcStructuralReaction = exports.IfcStructuralPointReaction = exports.IfcStructuralPointAction = exports.IfcStructuralPlanarAction = exports.IfcStructuralMember = exports.IfcStructuralLoadTemperature = exports.IfcStructuralLoadStatic = exports.IfcStructuralLoadSingleForceWarping = exports.IfcStructuralLoadSingleForce = exports.IfcStructuralLoadSingleDisplacementDistortion = exports.IfcStructuralLoadSingleDisplacement = exports.IfcStructuralLoadPlanarForce = exports.IfcStructuralLoadOrResult = exports.IfcStructuralLoadLinearForce = exports.IfcStructuralLoadGroup = exports.IfcStructuralLoadCase = exports.IfcStructuralLoad = exports.IfcStructuralLinearAction = exports.IfcStructuralItem = exports.IfcStructuralCurveReaction = exports.IfcStructuralCurveMemberVarying = exports.IfcStructuralCurveAction = exports.IfcStructuralConnectionCondition = exports.IfcStructuralAction = exports.IfcStairType = exports.IfcStairFlightType = exports.IfcStairFlight = exports.IfcStair = exports.IfcStackTerminalType = exports.IfcStackTerminal = exports.IfcSphericalSurface = exports.IfcSphere = exports.IfcSpatialZoneType = exports.IfcSpatialZone = exports.IfcSpatialStructureElementType = exports.IfcSpatialStructureElement = exports.IfcSpatialElementType = exports.IfcSpaceType = exports.IfcSpaceHeaterType = exports.IfcSpaceHeater = exports.IfcSpace = exports.IfcSolidModel = exports.IfcSolarDeviceType = exports.IfcSolarDevice = exports.IfcSlippageConnectionCondition = exports.IfcSlabType = exports.IfcSlabStandardCase = exports.IfcSlabElementedCase = exports.IfcSlab = exports.IfcSimpleProperty = exports.IfcShapeRepresentation = exports.IfcShapeModel = exports.IfcShadingDeviceType = exports.IfcShadingDevice = exports.IfcSensorType = exports.IfcSensor = exports.IfcSeamCurve = exports.IfcSchedulingTime = exports.IfcSanitaryTerminalType = exports.IfcSanitaryTerminal = exports.IfcSIUnit = exports.IfcRoundedRectangleProfileDef = exports.IfcRoofType = exports.IfcRoof = exports.IfcRightCircularCylinder = exports.IfcRightCircularCone = exports.IfcResourceTime = exports.IfcResourceLevelRelationship = exports.IfcResource = exports.IfcRepresentationContext = exports.IfcReparametrisedCompositeCurveSegment = exports.IfcRelationship = exports.IfcRelDefines = exports.IfcRelDecomposes = exports.IfcRelConnectsPathElements = exports.IfcRelConnects = exports.IfcRelAssignsToGroupByFactor = exports.IfcReinforcingMesh = exports.IfcReinforcingElementType = exports.IfcReinforcingElement = exports.IfcReinforcingBar = exports.IfcReinforcementBarProperties = exports.IfcRectangularPyramid = exports.IfcRectangleProfileDef = exports.IfcRectangleHollowProfileDef = exports.IfcRationalBSplineCurveWithKnots = exports.IfcRampType = exports.IfcRampFlightType = exports.IfcRampFlight = exports.IfcRamp = exports.IfcRailingType = exports.IfcRailing = exports.IfcQuantityWeight = exports.IfcQuantityVolume = exports.IfcQuantityTime = exports.IfcQuantitySet = exports.IfcQuantityLength = exports.IfcQuantityCount = exports.IfcQuantityArea = exports.IfcPumpType = exports.IfcPump = exports.IfcProxy = exports.IfcProtectiveDeviceType = exports.IfcProtectiveDeviceTrippingUnitType = exports.IfcProtectiveDeviceTrippingUnit = exports.IfcProtectiveDevice = exports.IfcPropertyTemplateDefinition = exports.IfcPropertyTemplate = exports.IfcPropertySetDefinition = exports.IfcPropertyDefinition = exports.IfcPropertyAbstraction = exports.IfcProperty = exports.IfcProjectionElement = exports.IfcProjectOrder = exports.IfcProjectLibrary = exports.IfcProject = exports.IfcProfileProperties = exports.IfcProcess = exports.IfcProcedureType = exports.IfcProcedure = exports.IfcPresentationStyle = exports.IfcPresentationItem = exports.IfcPreDefinedTextFont = exports.IfcPreDefinedPropertySet = exports.IfcPreDefinedProperties = exports.IfcPreDefinedItem = exports.IfcPreDefinedCurveFont = exports.IfcPreDefinedColour = exports.IfcPort = exports.IfcPoint = exports.IfcPlateType = exports.IfcPlateStandardCase = exports.IfcPlate = exports.IfcPlane = exports.IfcPlanarExtent = exports.IfcPixelTexture = exports.IfcPipeSegmentType = exports.IfcPipeSegment = exports.IfcPipeFittingType = exports.IfcPipeFitting = exports.IfcPileType = exports.IfcPile = exports.IfcPhysicalQuantity = exports.IfcPermit = exports.IfcPerformanceHistory = exports.IfcOutletType = exports.IfcOutlet = exports.IfcOuterBoundaryCurve = exports.IfcOrientedEdge = exports.IfcOpeningStandardCase = exports.IfcOpenShell = exports.IfcOccupant = exports.IfcObjectPlacement = exports.IfcMotorConnectionType = exports.IfcMotorConnection = exports.IfcMonetaryUnit = exports.IfcMirroredProfileDef = exports.IfcMemberType = exports.IfcMemberStandardCase = exports.IfcMember = exports.IfcMedicalDeviceType = exports.IfcMedicalDevice = exports.IfcMechanicalFastenerType = exports.IfcMechanicalFastener = exports.IfcMaterialUsageDefinition = exports.IfcMaterialProperties = exports.IfcMaterialProfileWithOffsets = exports.IfcMaterialLayerWithOffsets = exports.IfcMaterialDefinitionRepresentation = exports.IfcMapConversion = exports.IfcLoop = exports.IfcLightSourceAmbient = exports.IfcLightFixtureType = exports.IfcLightFixture = exports.IfcLightDistributionData = exports.IfcLampType = exports.IfcLamp = exports.IfcLaborResourceType = exports.IfcLaborResource = exports.IfcLShapeProfileDef = exports.IfcJunctionBoxType = exports.IfcJunctionBox = exports.IfcIntersectionCurve = exports.IfcInterceptorType = exports.IfcInterceptor = exports.IfcIndexedTriangleTextureMap = exports.IfcIndexedPolygonalFace = exports.IfcImageTexture = exports.IfcIShapeProfileDef = exports.IfcHumidifierType = exports.IfcHumidifier = exports.IfcHeatExchangerType = exports.IfcHeatExchanger = exports.IfcGroup = exports.IfcGeometricRepresentationSubContext = exports.IfcGeometricRepresentationItem = exports.IfcGeometricCurveSet = exports.IfcGeographicElementType = exports.IfcGeographicElement = exports.IfcFurnitureType = exports.IfcFurniture = exports.IfcFurnishingElementType = exports.IfcFurnishingElement = exports.IfcFootingType = exports.IfcFooting = exports.IfcFlowTreatmentDeviceType = exports.IfcFlowTreatmentDevice = exports.IfcFlowTerminalType = exports.IfcFlowTerminal = exports.IfcFlowStorageDeviceType = exports.IfcFlowStorageDevice = exports.IfcFlowSegmentType = exports.IfcFlowSegment = exports.IfcFlowMovingDeviceType = exports.IfcFlowMovingDevice = exports.IfcFlowMeterType = exports.IfcFlowMeter = exports.IfcFlowInstrumentType = exports.IfcFlowInstrument = exports.IfcFlowFittingType = exports.IfcFlowFitting = exports.IfcFlowControllerType = exports.IfcFlowController = exports.IfcFireSuppressionTerminalType = exports.IfcFireSuppressionTerminal = exports.IfcFilterType = exports.IfcFilter = exports.TilingPattern = exports.IfcFeatureElementSubtraction = exports.IfcFeatureElementAddition = exports.IfcFeatureElement = exports.IfcFastenerType = exports.IfcFastener = exports.IfcFanType = exports.IfcFan = exports.IfcFailureConnectionCondition = exports.IfcFacetedBrep = exports.IfcFaceOuterBound = exports.IfcExternallyDefinedTextFont = exports.IfcExternallyDefinedSurfaceStyle = exports.IfcExternallyDefinedHatchStyle = exports.IfcExternalSpatialStructureElement = exports.IfcExternalSpatialElement = exports.IfcExternalReference = exports.IfcExternalInformation = exports.IfcEventType = exports.IfcEventTime = exports.IfcEvaporatorType = exports.IfcEvaporator = exports.IfcEvaporativeCoolerType = exports.IfcEvaporativeCooler = exports.IfcEngineType = exports.IfcEngine = exports.IfcEnergyConversionDeviceType = exports.IfcEnergyConversionDevice = exports.IfcEllipseProfileDef = exports.IfcEllipse = exports.IfcElementType = exports.IfcElementComponentType = exports.IfcElementComponent = exports.IfcElementAssemblyType = exports.IfcElementAssembly = exports.IfcElectricTimeControlType = exports.IfcElectricTimeControl = exports.IfcElectricMotorType = exports.IfcElectricMotor = exports.IfcElectricGeneratorType = exports.IfcElectricGenerator = exports.IfcElectricFlowStorageDeviceType = exports.IfcElectricFlowStorageDevice = exports.IfcElectricDistributionBoardType = exports.IfcElectricDistributionBoard = exports.IfcElectricApplianceType = exports.IfcElectricAppliance = exports.IfcDuctSilencerType = exports.IfcDuctSilencer = exports.IfcDuctSegmentType = exports.IfcDuctSegment = exports.IfcDuctFittingType = exports.IfcDuctFitting = exports.IfcDraughtingPreDefinedCurveFont = exports.IfcDraughtingPreDefinedColour = exports.IfcDoorType = exports.IfcDoorStyle = exports.IfcDoorStandardCase = exports.IfcDoor = exports.IfcDistributionSystem = exports.IfcDistributionPort = exports.IfcDistributionFlowElementType = exports.IfcDistributionFlowElement = exports.IfcDistributionElementType = exports.IfcDistributionElement = exports.IfcDistributionControlElementType = exports.IfcDistributionControlElement = exports.IfcDistributionCircuit = exports.IfcDistributionChamberElementType = exports.IfcDistributionChamberElement = exports.IfcDiscreteAccessoryType = exports.IfcDiscreteAccessory = exports.IfcDirection = exports.IfcDimensionalExponents = exports.IfcDamperType = exports.IfcDamper = exports.IfcCylindricalSurface = exports.IfcCurveStyleFontPattern = exports.IfcCurve = exports.IfcCurtainWallType = exports.IfcCurtainWall = exports.IfcCrewResourceType = exports.IfcCrewResource = exports.IfcCoveringType = exports.IfcCovering = exports.IfcCostValue = exports.IfcCostSchedule = exports.IfcCoordinateReferenceSystem = exports.IfcCoolingTowerType = exports.IfcCoolingTower = exports.IfcCooledBeamType = exports.IfcCooledBeam = exports.IfcConversionBasedUnitWithOffset = exports.IfcControllerType = exports.IfcController = exports.IfcControl = exports.IfcContextDependentUnit = exports.IfcConstructionProductResourceType = exports.IfcConstructionProductResource = exports.IfcConstructionMaterialResourceType = exports.IfcConstructionMaterialResource = exports.IfcConstructionEquipmentResourceType = exports.IfcConstructionEquipmentResource = exports.IfcConnectionPointEccentricity = exports.IfcConnectionGeometry = exports.IfcCondenserType = exports.IfcCondenser = exports.IfcCompressorType = exports.IfcCompressor = exports.IfcCompositeCurveOnSurface = exports.IfcCommunicationsApplianceType = exports.IfcCommunicationsAppliance = exports.IfcColumnType = exports.IfcColumnStandardCase = exports.IfcColumn = exports.IfcColourSpecification = exports.IfcColourRgbList = exports.IfcColourRgb = exports.IfcCoilType = exports.IfcCoil = exports.IfcClosedShell = exports.IfcCivilElementType = exports.IfcCivilElement = exports.IfcCircleProfileDef = exports.IfcCircleHollowProfileDef = exports.IfcCircle = exports.IfcChimneyType = exports.IfcChimney = exports.IfcChillerType = exports.IfcChiller = exports.IfcCenterLineProfileDef = exports.IfcCartesianTransformationOperator3DnonUniform = exports.IfcCartesianTransformationOperator2DnonUniform = exports.IfcCartesianTransformationOperator2D = exports.IfcCartesianPointList3D = exports.IfcCartesianPointList2D = exports.IfcCartesianPointList = exports.IfcCartesianPoint = exports.IfcCableSegmentType = exports.IfcCableSegment = exports.IfcCableFittingType = exports.IfcCableFitting = exports.IfcCableCarrierSegmentType = exports.IfcCableCarrierSegment = exports.IfcCableCarrierFittingType = exports.IfcCableCarrierFitting = exports.IfcCShapeProfileDef = exports.IfcBurnerType = exports.IfcBurner = exports.IfcBuildingSystem = exports.IfcBuildingStorey = exports.IfcBuildingElementType = exports.IfcBuildingElementProxyType = exports.IfcBuildingElementProxy = exports.IfcBuildingElementPartType = exports.IfcBuildingElementPart = exports.IfcBuildingElement = exports.IfcBoundedSurface = exports.IfcBoundedCurve = exports.IfcBoundaryCurve = exports.IfcBoundaryCondition = exports.IfcBooleanClippingResult = exports.IfcBoilerType = exports.IfcBoiler = exports.IfcBlock = exports.IfcBeamType = exports.IfcBeamStandardCase = exports.IfcBeam = exports.IfcBSplineSurfaceWithKnots = exports.IfcBSplineCurveWithKnots = exports.IfcAudioVisualApplianceType = exports.IfcAudioVisualAppliance = exports.IfcAsymmetricIShapeProfileDef = exports.IfcAnnotation = exports.IfcAlarmType = exports.IfcAlarm = exports.IfcAirToAirHeatRecoveryType = exports.IfcAirToAirHeatRecovery = exports.IfcAirTerminalType = exports.IfcAirTerminalBoxType = exports.IfcAirTerminalBox = exports.IfcAirTerminal = exports.IfcAdvancedFace = exports.IfcAdvancedBrep = exports.IfcAddress = exports.IfcActuatorType = exports.IfcActuator = exports.IfcActorRole = exports.IfcActionRequest = exports.IfcXML = exports.Header = exports.Uos = exports.InstanceAttributes = exports.IFC4 = void 0;
var IFC4 = /** @class */ (function () {
    function IFC4() {
    }
    return IFC4;
}());
exports.IFC4 = IFC4;
-IfcPositiveInteger[];
ifcIndexedTextureMap: IfcIndexedTextureMap;
ifcIndexedTriangleTextureMap: IfcIndexedTriangleTextureMap;
ifcInterceptor: IfcInterceptor;
ifcInterceptorType: IfcInterceptorType;
ifcIntersectionCurve: IfcIntersectionCurve;
ifcInventory: IfcInventory;
ifcIrregularTimeSeries: IfcIrregularTimeSeries;
ifcIrregularTimeSeriesValue: IfcIrregularTimeSeriesValue;
ifcJunctionBox: IfcJunctionBox;
ifcJunctionBoxType: IfcJunctionBoxType;
ifcLShapeProfileDef: IfcLShapeProfileDef;
ifcLaborResource: IfcLaborResource;
ifcLaborResourceType: IfcLaborResourceType;
ifcLagTime: IfcLagTime;
ifcLamp: IfcLamp;
ifcLampType: IfcLampType;
ifcLibraryInformation: IfcLibraryInformation;
ifcLibraryReference: IfcLibraryReference;
ifcLightDistributionData: IfcLightDistributionData;
ifcLightFixture: IfcLightFixture;
ifcLightFixtureType: IfcLightFixtureType;
ifcLightIntensityDistribution: IfcLightIntensityDistribution;
ifcLightSource: IfcLightSource;
ifcLightSourceAmbient: IfcLightSourceAmbient;
ifcLightSourceDirectional: IfcLightSourceDirectional;
ifcLightSourceGoniometric: IfcLightSourceGoniometric;
ifcLightSourcePositional: IfcLightSourcePositional;
ifcLightSourceSpot: IfcLightSourceSpot;
ifcLine: IfcLine;
ifcLocalPlacement: IfcLocalPlacement;
ifcLoop: IfcLoop;
ifcManifoldSolidBrep: IfcManifoldSolidBrep;
ifcMapConversion: IfcMapConversion;
ifcMappedItem: IfcMappedItem;
ifcMaterial: IfcMaterial;
ifcMaterialClassificationRelationship: IfcMaterialClassificationRelationship;
ifcMaterialConstituent: IfcMaterialConstituent;
ifcMaterialConstituentSet: IfcMaterialConstituentSet;
ifcMaterialDefinition: IfcMaterialDefinition;
ifcMaterialDefinitionRepresentation: IfcMaterialDefinitionRepresentation;
ifcMaterialLayer: IfcMaterialLayer;
ifcMaterialLayerSet: IfcMaterialLayerSet;
ifcMaterialLayerSetUsage: IfcMaterialLayerSetUsage;
ifcMaterialLayerWithOffsets: IfcMaterialLayerWithOffsets;
ifcMaterialList: IfcMaterialList;
ifcMaterialProfile: IfcMaterialProfile;
ifcMaterialProfileSet: IfcMaterialProfileSet;
ifcMaterialProfileSetUsage: IfcMaterialProfileSetUsage;
ifcMaterialProfileSetUsageTapering: IfcMaterialProfileSetUsageTapering;
ifcMaterialProfileWithOffsets: IfcMaterialProfileWithOffsets;
ifcMaterialProperties: IfcMaterialProperties;
ifcMaterialRelationship: IfcMaterialRelationship;
ifcMaterialUsageDefinition: IfcMaterialUsageDefinition;
ifcMeasureWithUnit: IfcMeasureWithUnit;
ifcMechanicalFastener: IfcMechanicalFastener;
ifcMechanicalFastenerType: IfcMechanicalFastenerType;
ifcMedicalDevice: IfcMedicalDevice;
ifcMedicalDeviceType: IfcMedicalDeviceType;
ifcMember: IfcMember;
ifcMemberStandardCase: IfcMemberStandardCase;
ifcMemberType: IfcMemberType;
ifcMetric: IfcMetric;
ifcMirroredProfileDef: IfcMirroredProfileDef;
ifcMonetaryUnit: IfcMonetaryUnit;
ifcMotorConnection: IfcMotorConnection;
ifcMotorConnectionType: IfcMotorConnectionType;
ifcNamedUnit: IfcNamedUnit;
ifcObject: IfcObject;
ifcObjectDefinition: IfcObjectDefinition;
ifcObjectPlacement: IfcObjectPlacement;
ifcObjective: IfcObjective;
ifcOccupant: IfcOccupant;
ifcOffsetCurve2D: IfcOffsetCurve2D;
ifcOffsetCurve3D: IfcOffsetCurve3D;
ifcOpenShell: IfcOpenShell;
ifcOpeningElement: IfcOpeningElement;
ifcOpeningStandardCase: IfcOpeningStandardCase;
ifcOrganization: IfcOrganization;
ifcOrganizationRelationship: IfcOrganizationRelationship;
ifcOrientedEdge: IfcOrientedEdge;
ifcOuterBoundaryCurve: IfcOuterBoundaryCurve;
ifcOutlet: IfcOutlet;
ifcOutletType: IfcOutletType;
ifcOwnerHistory: IfcOwnerHistory;
ifcParameterizedProfileDef: IfcParameterizedProfileDef;
ifcPath: IfcPath;
ifcPcurve: IfcPcurve;
ifcPerformanceHistory: IfcPerformanceHistory;
ifcPermeableCoveringProperties: IfcPermeableCoveringProperties;
ifcPermit: IfcPermit;
ifcPerson: IfcPerson;
ifcPersonAndOrganization: IfcPersonAndOrganization;
ifcPhysicalComplexQuantity: IfcPhysicalComplexQuantity;
ifcPhysicalQuantity: IfcPhysicalQuantity;
ifcPhysicalSimpleQuantity: IfcPhysicalSimpleQuantity;
ifcPile: IfcPile;
ifcPileType: IfcPileType;
ifcPipeFitting: IfcPipeFitting;
ifcPipeFittingType: IfcPipeFittingType;
ifcPipeSegment: IfcPipeSegment;
ifcPipeSegmentType: IfcPipeSegmentType;
ifcPixelTexture: IfcPixelTexture;
ifcPlacement: IfcPlacement;
ifcPlanarBox: IfcPlanarBox;
ifcPlanarExtent: IfcPlanarExtent;
ifcPlane: IfcPlane;
ifcPlate: IfcPlate;
ifcPlateStandardCase: IfcPlateStandardCase;
ifcPlateType: IfcPlateType;
ifcPoint: IfcPoint;
ifcPointOnCurve: IfcPointOnCurve;
ifcPointOnSurface: IfcPointOnSurface;
ifcPolyLoop: IfcPolyLoop;
ifcPolygonalBoundedHalfSpace: IfcPolygonalBoundedHalfSpace;
ifcPolygonalFaceSet: IfcPolygonalFaceSet;
ifcPolyline: IfcPolyline;
ifcPort: IfcPort;
ifcPostalAddress: IfcPostalAddress;
ifcPreDefinedColour: IfcPreDefinedColour;
ifcPreDefinedCurveFont: IfcPreDefinedCurveFont;
ifcPreDefinedItem: IfcPreDefinedItem;
ifcPreDefinedProperties: IfcPreDefinedProperties;
ifcPreDefinedPropertySet: IfcPreDefinedPropertySet;
ifcPreDefinedTextFont: IfcPreDefinedTextFont;
ifcPresentationItem: IfcPresentationItem;
ifcPresentationLayerAssignment: IfcPresentationLayerAssignment;
ifcPresentationLayerWithStyle: IfcPresentationLayerWithStyle;
ifcPresentationStyle: IfcPresentationStyle;
ifcPresentationStyleAssignment: IfcPresentationStyleAssignment;
ifcProcedure: IfcProcedure;
ifcProcedureType: IfcProcedureType;
ifcProcess: IfcProcess;
ifcProduct: IfcProduct;
ifcProductDefinitionShape: IfcProductDefinitionShape;
ifcProductRepresentation: IfcProductRepresentation;
ifcProfileDef: IfcProfileDef;
ifcProfileProperties: IfcProfileProperties;
ifcProject: IfcProject;
ifcProjectLibrary: IfcProjectLibrary;
ifcProjectOrder: IfcProjectOrder;
ifcProjectedCRS: IfcProjectedCRS;
ifcProjectionElement: IfcProjectionElement;
ifcProperty: IfcProperty;
ifcPropertyAbstraction: IfcPropertyAbstraction;
ifcPropertyBoundedValue: IfcPropertyBoundedValue;
ifcPropertyDefinition: IfcPropertyDefinition;
ifcPropertyDependencyRelationship: IfcPropertyDependencyRelationship;
ifcPropertyEnumeratedValue: IfcPropertyEnumeratedValue;
ifcPropertyEnumeration: IfcPropertyEnumeration;
ifcPropertyListValue: IfcPropertyListValue;
ifcPropertyReferenceValue: IfcPropertyReferenceValue;
ifcPropertySet: IfcPropertySet;
ifcPropertySetDefinition: IfcPropertySetDefinition;
ifcPropertySetTemplate: IfcPropertySetTemplate;
ifcPropertySingleValue: IfcPropertySingleValue;
ifcPropertyTableValue: IfcPropertyTableValue;
ifcPropertyTemplate: IfcPropertyTemplate;
ifcPropertyTemplateDefinition: IfcPropertyTemplateDefinition;
ifcProtectiveDevice: IfcProtectiveDevice;
ifcProtectiveDeviceTrippingUnit: IfcProtectiveDeviceTrippingUnit;
ifcProtectiveDeviceTrippingUnitType: IfcProtectiveDeviceTrippingUnitType;
ifcProtectiveDeviceType: IfcProtectiveDeviceType;
ifcProxy: IfcProxy;
ifcPump: IfcPump;
ifcPumpType: IfcPumpType;
ifcQuantityArea: IfcQuantityArea;
ifcQuantityCount: IfcQuantityCount;
ifcQuantityLength: IfcQuantityLength;
ifcQuantitySet: IfcQuantitySet;
ifcQuantityTime: IfcQuantityTime;
ifcQuantityVolume: IfcQuantityVolume;
ifcQuantityWeight: IfcQuantityWeight;
ifcRailing: IfcRailing;
ifcRailingType: IfcRailingType;
ifcRamp: IfcRamp;
ifcRampFlight: IfcRampFlight;
ifcRampFlightType: IfcRampFlightType;
ifcRampType: IfcRampType;
ifcRationalBSplineCurveWithKnots: IfcRationalBSplineCurveWithKnots;
ifcRationalBSplineSurfaceWithKnots: IfcRationalBSplineSurfaceWithKnots;
ifcRectangleHollowProfileDef: IfcRectangleHollowProfileDef;
ifcRectangleProfileDef: IfcRectangleProfileDef;
ifcRectangularPyramid: IfcRectangularPyramid;
ifcRectangularTrimmedSurface: IfcRectangularTrimmedSurface;
ifcRecurrencePattern: IfcRecurrencePattern;
ifcReference: IfcReference;
ifcRegularTimeSeries: IfcRegularTimeSeries;
ifcReinforcementBarProperties: IfcReinforcementBarProperties;
ifcReinforcementDefinitionProperties: IfcReinforcementDefinitionProperties;
ifcReinforcingBar: IfcReinforcingBar;
ifcReinforcingBarType: IfcReinforcingBarType;
ifcReinforcingElement: IfcReinforcingElement;
ifcReinforcingElementType: IfcReinforcingElementType;
ifcReinforcingMesh: IfcReinforcingMesh;
ifcReinforcingMeshType: IfcReinforcingMeshType;
ifcRelAggregates: IfcRelAggregates;
ifcRelAssigns: IfcRelAssigns;
ifcRelAssignsToActor: IfcRelAssignsToActor;
ifcRelAssignsToControl: IfcRelAssignsToControl;
ifcRelAssignsToGroup: IfcRelAssignsToGroup;
ifcRelAssignsToGroupByFactor: IfcRelAssignsToGroupByFactor;
ifcRelAssignsToProcess: IfcRelAssignsToProcess;
ifcRelAssignsToProduct: IfcRelAssignsToProduct;
ifcRelAssignsToResource: IfcRelAssignsToResource;
ifcRelAssociates: IfcRelAssociates;
ifcRelAssociatesApproval: IfcRelAssociatesApproval;
ifcRelAssociatesClassification: IfcRelAssociatesClassification;
ifcRelAssociatesConstraint: IfcRelAssociatesConstraint;
ifcRelAssociatesDocument: IfcRelAssociatesDocument;
ifcRelAssociatesLibrary: IfcRelAssociatesLibrary;
ifcRelAssociatesMaterial: IfcRelAssociatesMaterial;
ifcRelConnects: IfcRelConnects;
ifcRelConnectsElements: IfcRelConnectsElements;
ifcRelConnectsPathElements: IfcRelConnectsPathElements;
ifcRelConnectsPortToElement: IfcRelConnectsPortToElement;
ifcRelConnectsPorts: IfcRelConnectsPorts;
ifcRelConnectsStructuralActivity: IfcRelConnectsStructuralActivity;
ifcRelConnectsStructuralMember: IfcRelConnectsStructuralMember;
ifcRelConnectsWithEccentricity: IfcRelConnectsWithEccentricity;
ifcRelConnectsWithRealizingElements: IfcRelConnectsWithRealizingElements;
ifcRelContainedInSpatialStructure: IfcRelContainedInSpatialStructure;
ifcRelCoversBldgElements: IfcRelCoversBldgElements;
ifcRelCoversSpaces: IfcRelCoversSpaces;
ifcRelDeclares: IfcRelDeclares;
ifcRelDecomposes: IfcRelDecomposes;
ifcRelDefines: IfcRelDefines;
ifcRelDefinesByObject: IfcRelDefinesByObject;
ifcRelDefinesByProperties: IfcRelDefinesByProperties;
ifcRelDefinesByTemplate: IfcRelDefinesByTemplate;
ifcRelDefinesByType: IfcRelDefinesByType;
ifcRelFillsElement: IfcRelFillsElement;
ifcRelFlowControlElements: IfcRelFlowControlElements;
ifcRelInterferesElements: IfcRelInterferesElements;
ifcRelNests: IfcRelNests;
ifcRelProjectsElement: IfcRelProjectsElement;
ifcRelReferencedInSpatialStructure: IfcRelReferencedInSpatialStructure;
ifcRelSequence: IfcRelSequence;
ifcRelServicesBuildings: IfcRelServicesBuildings;
ifcRelSpaceBoundary: IfcRelSpaceBoundary;
ifcRelSpaceBoundary1stLevel: IfcRelSpaceBoundary1stLevel;
ifcRelSpaceBoundary2ndLevel: IfcRelSpaceBoundary2ndLevel;
ifcRelVoidsElement: IfcRelVoidsElement;
ifcRelationship: IfcRelationship;
ifcReparametrisedCompositeCurveSegment: IfcReparametrisedCompositeCurveSegment;
ifcRepresentation: IfcRepresentation;
ifcRepresentationContext: IfcRepresentationContext;
ifcRepresentationItem: IfcRepresentationItem;
ifcRepresentationMap: IfcRepresentationMap;
ifcResource: IfcResource;
ifcResourceApprovalRelationship: IfcResourceApprovalRelationship;
ifcResourceConstraintRelationship: IfcResourceConstraintRelationship;
ifcResourceLevelRelationship: IfcResourceLevelRelationship;
ifcResourceTime: IfcResourceTime;
ifcRevolvedAreaSolid: IfcRevolvedAreaSolid;
ifcRevolvedAreaSolidTapered: IfcRevolvedAreaSolidTapered;
ifcRightCircularCone: IfcRightCircularCone;
ifcRightCircularCylinder: IfcRightCircularCylinder;
ifcRoof: IfcRoof;
ifcRoofType: IfcRoofType;
ifcRoot: IfcRoot;
ifcRoundedRectangleProfileDef: IfcRoundedRectangleProfileDef;
ifcSIUnit: IfcSIUnit;
ifcSanitaryTerminal: IfcSanitaryTerminal;
ifcSanitaryTerminalType: IfcSanitaryTerminalType;
ifcSchedulingTime: IfcSchedulingTime;
ifcSeamCurve: IfcSeamCurve;
ifcSectionProperties: IfcSectionProperties;
ifcSectionReinforcementProperties: IfcSectionReinforcementProperties;
ifcSectionedSpine: IfcSectionedSpine;
ifcSensor: IfcSensor;
ifcSensorType: IfcSensorType;
ifcShadingDevice: IfcShadingDevice;
ifcShadingDeviceType: IfcShadingDeviceType;
ifcShapeAspect: IfcShapeAspect;
ifcShapeModel: IfcShapeModel;
ifcShapeRepresentation: IfcShapeRepresentation;
ifcShellBasedSurfaceModel: IfcShellBasedSurfaceModel;
ifcSimpleProperty: IfcSimpleProperty;
ifcSimplePropertyTemplate: IfcSimplePropertyTemplate;
ifcSite: IfcSite;
ifcSlab: IfcSlab;
ifcSlabElementedCase: IfcSlabElementedCase;
ifcSlabStandardCase: IfcSlabStandardCase;
ifcSlabType: IfcSlabType;
ifcSlippageConnectionCondition: IfcSlippageConnectionCondition;
ifcSolarDevice: IfcSolarDevice;
ifcSolarDeviceType: IfcSolarDeviceType;
ifcSolidModel: IfcSolidModel;
ifcSpace: IfcSpace;
ifcSpaceHeater: IfcSpaceHeater;
ifcSpaceHeaterType: IfcSpaceHeaterType;
ifcSpaceType: IfcSpaceType;
ifcSpatialElement: IfcSpatialElement;
ifcSpatialElementType: IfcSpatialElementType;
ifcSpatialStructureElement: IfcSpatialStructureElement;
ifcSpatialStructureElementType: IfcSpatialStructureElementType;
ifcSpatialZone: IfcSpatialZone;
ifcSpatialZoneType: IfcSpatialZoneType;
ifcSphere: IfcSphere;
ifcSphericalSurface: IfcSphericalSurface;
ifcStackTerminal: IfcStackTerminal;
ifcStackTerminalType: IfcStackTerminalType;
ifcStair: IfcStair;
ifcStairFlight: IfcStairFlight;
ifcStairFlightType: IfcStairFlightType;
ifcStairType: IfcStairType;
ifcStructuralAction: IfcStructuralAction;
ifcStructuralActivity: IfcStructuralActivity;
ifcStructuralAnalysisModel: IfcStructuralAnalysisModel;
ifcStructuralConnection: IfcStructuralConnection;
ifcStructuralConnectionCondition: IfcStructuralConnectionCondition;
ifcStructuralCurveAction: IfcStructuralCurveAction;
ifcStructuralCurveConnection: IfcStructuralCurveConnection;
ifcStructuralCurveMember: IfcStructuralCurveMember;
ifcStructuralCurveMemberVarying: IfcStructuralCurveMemberVarying;
ifcStructuralCurveReaction: IfcStructuralCurveReaction;
ifcStructuralItem: IfcStructuralItem;
ifcStructuralLinearAction: IfcStructuralLinearAction;
ifcStructuralLoad: IfcStructuralLoad;
ifcStructuralLoadCase: IfcStructuralLoadCase;
ifcStructuralLoadConfiguration: IfcStructuralLoadConfiguration;
ifcStructuralLoadGroup: IfcStructuralLoadGroup;
ifcStructuralLoadLinearForce: IfcStructuralLoadLinearForce;
ifcStructuralLoadOrResult: IfcStructuralLoadOrResult;
ifcStructuralLoadPlanarForce: IfcStructuralLoadPlanarForce;
ifcStructuralLoadSingleDisplacement: IfcStructuralLoadSingleDisplacement;
ifcStructuralLoadSingleDisplacementDistortion: IfcStructuralLoadSingleDisplacementDistortion;
ifcStructuralLoadSingleForce: IfcStructuralLoadSingleForce;
ifcStructuralLoadSingleForceWarping: IfcStructuralLoadSingleForceWarping;
ifcStructuralLoadStatic: IfcStructuralLoadStatic;
ifcStructuralLoadTemperature: IfcStructuralLoadTemperature;
ifcStructuralMember: IfcStructuralMember;
ifcStructuralPlanarAction: IfcStructuralPlanarAction;
ifcStructuralPointAction: IfcStructuralPointAction;
ifcStructuralPointConnection: IfcStructuralPointConnection;
ifcStructuralPointReaction: IfcStructuralPointReaction;
ifcStructuralReaction: IfcStructuralReaction;
ifcStructuralResultGroup: IfcStructuralResultGroup;
ifcStructuralSurfaceAction: IfcStructuralSurfaceAction;
ifcStructuralSurfaceConnection: IfcStructuralSurfaceConnection;
ifcStructuralSurfaceMember: IfcStructuralSurfaceMember;
ifcStructuralSurfaceMemberVarying: IfcStructuralSurfaceMemberVarying;
ifcStructuralSurfaceReaction: IfcStructuralSurfaceReaction;
ifcStyleModel: IfcStyleModel;
ifcStyledItem: IfcStyledItem;
ifcStyledRepresentation: IfcStyledRepresentation;
ifcSubContractResource: IfcSubContractResource;
ifcSubContractResourceType: IfcSubContractResourceType;
ifcSubedge: IfcSubedge;
ifcSurface: IfcSurface;
ifcSurfaceCurve: IfcSurfaceCurve;
ifcSurfaceCurveSweptAreaSolid: IfcSurfaceCurveSweptAreaSolid;
ifcSurfaceFeature: IfcSurfaceFeature;
ifcSurfaceOfLinearExtrusion: IfcSurfaceOfLinearExtrusion;
ifcSurfaceOfRevolution: IfcSurfaceOfRevolution;
ifcSurfaceReinforcementArea: IfcSurfaceReinforcementArea;
ifcSurfaceStyle: IfcSurfaceStyle;
ifcSurfaceStyleLighting: IfcSurfaceStyleLighting;
ifcSurfaceStyleRefraction: IfcSurfaceStyleRefraction;
ifcSurfaceStyleRendering: IfcSurfaceStyleRendering;
ifcSurfaceStyleShading: IfcSurfaceStyleShading;
ifcSurfaceStyleWithTextures: IfcSurfaceStyleWithTextures;
ifcSurfaceTexture: IfcSurfaceTexture;
ifcSweptAreaSolid: IfcSweptAreaSolid;
ifcSweptDiskSolid: IfcSweptDiskSolid;
ifcSweptDiskSolidPolygonal: IfcSweptDiskSolidPolygonal;
ifcSweptSurface: IfcSweptSurface;
ifcSwitchingDevice: IfcSwitchingDevice;
ifcSwitchingDeviceType: IfcSwitchingDeviceType;
ifcSystem: IfcSystem;
ifcSystemFurnitureElement: IfcSystemFurnitureElement;
ifcSystemFurnitureElementType: IfcSystemFurnitureElementType;
ifcTShapeProfileDef: IfcTShapeProfileDef;
ifcTable: IfcTable;
ifcTableColumn: IfcTableColumn;
ifcTableRow: IfcTableRow;
ifcTank: IfcTank;
ifcTankType: IfcTankType;
ifcTask: IfcTask;
ifcTaskTime: IfcTaskTime;
ifcTaskTimeRecurring: IfcTaskTimeRecurring;
ifcTaskType: IfcTaskType;
ifcTelecomAddress: IfcTelecomAddress;
ifcTendon: IfcTendon;
ifcTendonAnchor: IfcTendonAnchor;
ifcTendonAnchorType: IfcTendonAnchorType;
ifcTendonType: IfcTendonType;
ifcTessellatedFaceSet: IfcTessellatedFaceSet;
ifcTessellatedItem: IfcTessellatedItem;
ifcTextLiteral: IfcTextLiteral;
ifcTextLiteralWithExtent: IfcTextLiteralWithExtent;
ifcTextStyle: IfcTextStyle;
ifcTextStyleFontModel: IfcTextStyleFontModel;
ifcTextStyleForDefinedFont: IfcTextStyleForDefinedFont;
ifcTextStyleTextModel: IfcTextStyleTextModel;
ifcTextureCoordinate: IfcTextureCoordinate;
ifcTextureCoordinateGenerator: IfcTextureCoordinateGenerator;
ifcTextureMap: IfcTextureMap;
ifcTextureVertex: IfcTextureVertex;
ifcTextureVertexList: IfcTextureVertexList;
ifcTimePeriod: IfcTimePeriod;
ifcTimeSeries: IfcTimeSeries;
ifcTimeSeriesValue: IfcTimeSeriesValue;
ifcTopologicalRepresentationItem: IfcTopologicalRepresentationItem;
ifcTopologyRepresentation: IfcTopologyRepresentation;
ifcToroidalSurface: IfcToroidalSurface;
ifcTransformer: IfcTransformer;
ifcTransformerType: IfcTransformerType;
ifcTransportElement: IfcTransportElement;
ifcTransportElementType: IfcTransportElementType;
ifcTrapeziumProfileDef: IfcTrapeziumProfileDef;
ifcTriangulatedFaceSet: IfcTriangulatedFaceSet;
ifcTrimmedCurve: IfcTrimmedCurve;
ifcTubeBundle: IfcTubeBundle;
ifcTubeBundleType: IfcTubeBundleType;
ifcTypeObject: IfcTypeObject;
ifcTypeProcess: IfcTypeProcess;
ifcTypeProduct: IfcTypeProduct;
ifcTypeResource: IfcTypeResource;
ifcUShapeProfileDef: IfcUShapeProfileDef;
ifcUnitAssignment: IfcUnitAssignment;
ifcUnitaryControlElement: IfcUnitaryControlElement;
ifcUnitaryControlElementType: IfcUnitaryControlElementType;
ifcUnitaryEquipment: IfcUnitaryEquipment;
ifcUnitaryEquipmentType: IfcUnitaryEquipmentType;
ifcValve: IfcValve;
ifcValveType: IfcValveType;
ifcVector: IfcVector;
ifcVertex: IfcVertex;
ifcVertexLoop: IfcVertexLoop;
ifcVertexPoint: IfcVertexPoint;
ifcVibrationIsolator: IfcVibrationIsolator;
ifcVibrationIsolatorType: IfcVibrationIsolatorType;
ifcVirtualElement: IfcVirtualElement;
ifcVirtualGridIntersection: IfcVirtualGridIntersection;
ifcVoidingFeature: IfcVoidingFeature;
ifcWall: IfcWall;
ifcWallElementedCase: IfcWallElementedCase;
ifcWallStandardCase: IfcWallStandardCase;
ifcWallType: IfcWallType;
ifcWasteTerminal: IfcWasteTerminal;
ifcWasteTerminalType: IfcWasteTerminalType;
ifcWindow: IfcWindow;
ifcWindowLiningProperties: IfcWindowLiningProperties;
ifcWindowPanelProperties: IfcWindowPanelProperties;
ifcWindowStandardCase: IfcWindowStandardCase;
ifcWindowStyle: IfcWindowStyle;
ifcWindowType: IfcWindowType;
ifcWorkCalendar: IfcWorkCalendar;
ifcWorkControl: IfcWorkControl;
ifcWorkPlan: IfcWorkPlan;
ifcWorkSchedule: IfcWorkSchedule;
ifcWorkTime: IfcWorkTime;
ifcZShapeProfileDef: IfcZShapeProfileDef;
ifcZone: IfcZone;
entity: Entity;
ifcAbsorbedDoseMeasure - wrapper;
IfcAbsorbedDoseMeasure - wrapper;
ifcAccelerationMeasure - wrapper;
IfcAccelerationMeasure - wrapper;
ifcAmountOfSubstanceMeasure - wrapper;
IfcAmountOfSubstanceMeasure - wrapper;
ifcAngularVelocityMeasure - wrapper;
IfcAngularVelocityMeasure - wrapper;
ifcArcIndex - wrapper;
IfcArcIndex - wrapper;
ifcAreaDensityMeasure - wrapper;
IfcAreaDensityMeasure - wrapper;
ifcAreaMeasure - wrapper;
IfcAreaMeasure - wrapper;
ifcBinary - wrapper;
IfcBinary - wrapper;
ifcBoolean - wrapper;
IfcBoolean - wrapper;
ifcComplexNumber - wrapper;
IfcComplexNumber - wrapper;
ifcCompoundPlaneAngleMeasure - wrapper;
IfcCompoundPlaneAngleMeasure - wrapper;
ifcContextDependentMeasure - wrapper;
IfcContextDependentMeasure - wrapper;
ifcCountMeasure - wrapper;
IfcCountMeasure - wrapper;
ifcCurvatureMeasure - wrapper;
IfcCurvatureMeasure - wrapper;
ifcDate - wrapper;
IfcDate - wrapper;
ifcDateTime - wrapper;
IfcDateTime - wrapper;
ifcDescriptiveMeasure - wrapper;
IfcDescriptiveMeasure - wrapper;
ifcDoseEquivalentMeasure - wrapper;
IfcDoseEquivalentMeasure - wrapper;
ifcDuration - wrapper;
IfcDuration - wrapper;
ifcDynamicViscosityMeasure - wrapper;
IfcDynamicViscosityMeasure - wrapper;
ifcElectricCapacitanceMeasure - wrapper;
IfcElectricCapacitanceMeasure - wrapper;
ifcElectricChargeMeasure - wrapper;
IfcElectricChargeMeasure - wrapper;
ifcElectricConductanceMeasure - wrapper;
IfcElectricConductanceMeasure - wrapper;
ifcElectricCurrentMeasure - wrapper;
IfcElectricCurrentMeasure - wrapper;
ifcElectricResistanceMeasure - wrapper;
IfcElectricResistanceMeasure - wrapper;
ifcElectricVoltageMeasure - wrapper;
IfcElectricVoltageMeasure - wrapper;
ifcEnergyMeasure - wrapper;
IfcEnergyMeasure - wrapper;
ifcForceMeasure - wrapper;
IfcForceMeasure - wrapper;
ifcFrequencyMeasure - wrapper;
IfcFrequencyMeasure - wrapper;
ifcHeatFluxDensityMeasure - wrapper;
IfcHeatFluxDensityMeasure - wrapper;
ifcHeatingValueMeasure - wrapper;
IfcHeatingValueMeasure - wrapper;
ifcIdentifier - wrapper;
IfcIdentifier - wrapper;
ifcIlluminanceMeasure - wrapper;
IfcIlluminanceMeasure - wrapper;
ifcInductanceMeasure - wrapper;
IfcInductanceMeasure - wrapper;
ifcInteger - wrapper;
IfcInteger - wrapper;
ifcIntegerCountRateMeasure - wrapper;
IfcIntegerCountRateMeasure - wrapper;
ifcIonConcentrationMeasure - wrapper;
IfcIonConcentrationMeasure - wrapper;
ifcIsothermalMoistureCapacityMeasure - wrapper;
IfcIsothermalMoistureCapacityMeasure - wrapper;
ifcKinematicViscosityMeasure - wrapper;
IfcKinematicViscosityMeasure - wrapper;
ifcLabel - wrapper;
IfcLabel - wrapper;
ifcLengthMeasure - wrapper;
IfcLengthMeasure - wrapper;
ifcLineIndex - wrapper;
IfcLineIndex - wrapper;
ifcLinearForceMeasure - wrapper;
IfcLinearForceMeasure - wrapper;
ifcLinearMomentMeasure - wrapper;
IfcLinearMomentMeasure - wrapper;
ifcLinearStiffnessMeasure - wrapper;
IfcLinearStiffnessMeasure - wrapper;
ifcLinearVelocityMeasure - wrapper;
IfcLinearVelocityMeasure - wrapper;
ifcLogical - wrapper;
IfcLogical - wrapper;
ifcLuminousFluxMeasure - wrapper;
IfcLuminousFluxMeasure - wrapper;
ifcLuminousIntensityDistributionMeasure - wrapper;
IfcLuminousIntensityDistributionMeasure - wrapper;
ifcLuminousIntensityMeasure - wrapper;
IfcLuminousIntensityMeasure - wrapper;
ifcMagneticFluxDensityMeasure - wrapper;
IfcMagneticFluxDensityMeasure - wrapper;
ifcMagneticFluxMeasure - wrapper;
IfcMagneticFluxMeasure - wrapper;
ifcMassDensityMeasure - wrapper;
IfcMassDensityMeasure - wrapper;
ifcMassFlowRateMeasure - wrapper;
IfcMassFlowRateMeasure - wrapper;
ifcMassMeasure - wrapper;
IfcMassMeasure - wrapper;
ifcMassPerLengthMeasure - wrapper;
IfcMassPerLengthMeasure - wrapper;
ifcModulusOfElasticityMeasure - wrapper;
IfcModulusOfElasticityMeasure - wrapper;
ifcModulusOfLinearSubgradeReactionMeasure - wrapper;
IfcModulusOfLinearSubgradeReactionMeasure - wrapper;
ifcModulusOfRotationalSubgradeReactionMeasure - wrapper;
IfcModulusOfRotationalSubgradeReactionMeasure - wrapper;
ifcModulusOfSubgradeReactionMeasure - wrapper;
IfcModulusOfSubgradeReactionMeasure - wrapper;
ifcMoistureDiffusivityMeasure - wrapper;
IfcMoistureDiffusivityMeasure - wrapper;
ifcMolecularWeightMeasure - wrapper;
IfcMolecularWeightMeasure - wrapper;
ifcMomentOfInertiaMeasure - wrapper;
IfcMomentOfInertiaMeasure - wrapper;
ifcMonetaryMeasure - wrapper;
IfcMonetaryMeasure - wrapper;
ifcNonNegativeLengthMeasure - wrapper;
IfcNonNegativeLengthMeasure - wrapper;
ifcNormalisedRatioMeasure - wrapper;
IfcNormalisedRatioMeasure - wrapper;
ifcNullStyle - wrapper;
IfcNullStyle - wrapper;
ifcNumericMeasure - wrapper;
IfcNumericMeasure - wrapper;
ifcPHMeasure - wrapper;
IfcPHMeasure - wrapper;
ifcParameterValue - wrapper;
IfcParameterValue - wrapper;
ifcPlanarForceMeasure - wrapper;
IfcPlanarForceMeasure - wrapper;
ifcPlaneAngleMeasure - wrapper;
IfcPlaneAngleMeasure - wrapper;
ifcPositiveInteger - wrapper;
IfcPositiveInteger - wrapper;
ifcPositiveLengthMeasure - wrapper;
IfcPositiveLengthMeasure - wrapper;
ifcPositivePlaneAngleMeasure - wrapper;
IfcPositivePlaneAngleMeasure - wrapper;
ifcPositiveRatioMeasure - wrapper;
IfcPositiveRatioMeasure - wrapper;
ifcPowerMeasure - wrapper;
IfcPowerMeasure - wrapper;
ifcPressureMeasure - wrapper;
IfcPressureMeasure - wrapper;
ifcPropertySetDefinitionSet - wrapper;
IfcPropertySetDefinitionSet - wrapper;
ifcRadioActivityMeasure - wrapper;
IfcRadioActivityMeasure - wrapper;
ifcRatioMeasure - wrapper;
IfcRatioMeasure - wrapper;
ifcReal - wrapper;
IfcReal - wrapper;
ifcRotationalFrequencyMeasure - wrapper;
IfcRotationalFrequencyMeasure - wrapper;
ifcRotationalMassMeasure - wrapper;
IfcRotationalMassMeasure - wrapper;
ifcRotationalStiffnessMeasure - wrapper;
IfcRotationalStiffnessMeasure - wrapper;
ifcSectionModulusMeasure - wrapper;
IfcSectionModulusMeasure - wrapper;
ifcSectionalAreaIntegralMeasure - wrapper;
IfcSectionalAreaIntegralMeasure - wrapper;
ifcShearModulusMeasure - wrapper;
IfcShearModulusMeasure - wrapper;
ifcSolidAngleMeasure - wrapper;
IfcSolidAngleMeasure - wrapper;
ifcSoundPowerLevelMeasure - wrapper;
IfcSoundPowerLevelMeasure - wrapper;
ifcSoundPowerMeasure - wrapper;
IfcSoundPowerMeasure - wrapper;
ifcSoundPressureLevelMeasure - wrapper;
IfcSoundPressureLevelMeasure - wrapper;
ifcSoundPressureMeasure - wrapper;
IfcSoundPressureMeasure - wrapper;
ifcSpecificHeatCapacityMeasure - wrapper;
IfcSpecificHeatCapacityMeasure - wrapper;
ifcSpecularExponent - wrapper;
IfcSpecularExponent - wrapper;
ifcSpecularRoughness - wrapper;
IfcSpecularRoughness - wrapper;
ifcTemperatureGradientMeasure - wrapper;
IfcTemperatureGradientMeasure - wrapper;
ifcTemperatureRateOfChangeMeasure - wrapper;
IfcTemperatureRateOfChangeMeasure - wrapper;
ifcText - wrapper;
IfcText - wrapper;
ifcTextFontName - wrapper;
IfcTextFontName - wrapper;
ifcThermalAdmittanceMeasure - wrapper;
IfcThermalAdmittanceMeasure - wrapper;
ifcThermalConductivityMeasure - wrapper;
IfcThermalConductivityMeasure - wrapper;
ifcThermalExpansionCoefficientMeasure - wrapper;
IfcThermalExpansionCoefficientMeasure - wrapper;
ifcThermalResistanceMeasure - wrapper;
IfcThermalResistanceMeasure - wrapper;
ifcThermalTransmittanceMeasure - wrapper;
IfcThermalTransmittanceMeasure - wrapper;
ifcThermodynamicTemperatureMeasure - wrapper;
IfcThermodynamicTemperatureMeasure - wrapper;
ifcTime - wrapper;
IfcTime - wrapper;
ifcTimeMeasure - wrapper;
IfcTimeMeasure - wrapper;
ifcTimeStamp - wrapper;
IfcTimeStamp - wrapper;
ifcTorqueMeasure - wrapper;
IfcTorqueMeasure - wrapper;
ifcVaporPermeabilityMeasure - wrapper;
IfcVaporPermeabilityMeasure - wrapper;
ifcVolumeMeasure - wrapper;
IfcVolumeMeasure - wrapper;
ifcVolumetricFlowRateMeasure - wrapper;
IfcVolumetricFlowRateMeasure - wrapper;
ifcWarpingConstantMeasure - wrapper;
IfcWarpingConstantMeasure - wrapper;
ifcWarpingMomentMeasure - wrapper;
IfcWarpingMomentMeasure - wrapper;
constructor(props ?  : IFC4);
{
    this["@class"] = ".IFC4";
    if (props) {
        this.uos = (props.uos) ? new Uos(props.uos) : undefined;
        this.ifcXML = (props.ifcXML) ? new IfcXML(props.ifcXML) : undefined;
        this.ifcActionRequest = (props.ifcActionRequest) ? new IfcActionRequest(props.ifcActionRequest) : undefined;
        this.ifcActor = (props.ifcActor) ? new IfcActor(props.ifcActor) : undefined;
        this.ifcActorRole = (props.ifcActorRole) ? new IfcActorRole(props.ifcActorRole) : undefined;
        this.ifcActuator = (props.ifcActuator) ? new IfcActuator(props.ifcActuator) : undefined;
        this.ifcActuatorType = (props.ifcActuatorType) ? new IfcActuatorType(props.ifcActuatorType) : undefined;
        this.ifcAddress = (props.ifcAddress) ? new IfcAddress(props.ifcAddress) : undefined;
        this.ifcAdvancedBrep = (props.ifcAdvancedBrep) ? new IfcAdvancedBrep(props.ifcAdvancedBrep) : undefined;
        this.ifcAdvancedBrepWithVoids = (props.ifcAdvancedBrepWithVoids) ? new IfcAdvancedBrepWithVoids(props.ifcAdvancedBrepWithVoids) : undefined;
        this.ifcAdvancedFace = (props.ifcAdvancedFace) ? new IfcAdvancedFace(props.ifcAdvancedFace) : undefined;
        this.ifcAirTerminal = (props.ifcAirTerminal) ? new IfcAirTerminal(props.ifcAirTerminal) : undefined;
        this.ifcAirTerminalBox = (props.ifcAirTerminalBox) ? new IfcAirTerminalBox(props.ifcAirTerminalBox) : undefined;
        this.ifcAirTerminalBoxType = (props.ifcAirTerminalBoxType) ? new IfcAirTerminalBoxType(props.ifcAirTerminalBoxType) : undefined;
        this.ifcAirTerminalType = (props.ifcAirTerminalType) ? new IfcAirTerminalType(props.ifcAirTerminalType) : undefined;
        this.ifcAirToAirHeatRecovery = (props.ifcAirToAirHeatRecovery) ? new IfcAirToAirHeatRecovery(props.ifcAirToAirHeatRecovery) : undefined;
        this.ifcAirToAirHeatRecoveryType = (props.ifcAirToAirHeatRecoveryType) ? new IfcAirToAirHeatRecoveryType(props.ifcAirToAirHeatRecoveryType) : undefined;
        this.ifcAlarm = (props.ifcAlarm) ? new IfcAlarm(props.ifcAlarm) : undefined;
        this.ifcAlarmType = (props.ifcAlarmType) ? new IfcAlarmType(props.ifcAlarmType) : undefined;
        this.ifcAnnotation = (props.ifcAnnotation) ? new IfcAnnotation(props.ifcAnnotation) : undefined;
        this.ifcAnnotationFillArea = (props.ifcAnnotationFillArea) ? new IfcAnnotationFillArea(props.ifcAnnotationFillArea) : undefined;
        this.ifcApplication = (props.ifcApplication) ? new IfcApplication(props.ifcApplication) : undefined;
        this.ifcAppliedValue = (props.ifcAppliedValue) ? new IfcAppliedValue(props.ifcAppliedValue) : undefined;
        this.ifcApproval = (props.ifcApproval) ? new IfcApproval(props.ifcApproval) : undefined;
        this.ifcApprovalRelationship = (props.ifcApprovalRelationship) ? new IfcApprovalRelationship(props.ifcApprovalRelationship) : undefined;
        this.ifcArbitraryClosedProfileDef = (props.ifcArbitraryClosedProfileDef) ? new IfcArbitraryClosedProfileDef(props.ifcArbitraryClosedProfileDef) : undefined;
        this.ifcArbitraryOpenProfileDef = (props.ifcArbitraryOpenProfileDef) ? new IfcArbitraryOpenProfileDef(props.ifcArbitraryOpenProfileDef) : undefined;
        this.ifcArbitraryProfileDefWithVoids = (props.ifcArbitraryProfileDefWithVoids) ? new IfcArbitraryProfileDefWithVoids(props.ifcArbitraryProfileDefWithVoids) : undefined;
        this.ifcAsset = (props.ifcAsset) ? new IfcAsset(props.ifcAsset) : undefined;
        this.ifcAsymmetricIShapeProfileDef = (props.ifcAsymmetricIShapeProfileDef) ? new IfcAsymmetricIShapeProfileDef(props.ifcAsymmetricIShapeProfileDef) : undefined;
        this.ifcAudioVisualAppliance = (props.ifcAudioVisualAppliance) ? new IfcAudioVisualAppliance(props.ifcAudioVisualAppliance) : undefined;
        this.ifcAudioVisualApplianceType = (props.ifcAudioVisualApplianceType) ? new IfcAudioVisualApplianceType(props.ifcAudioVisualApplianceType) : undefined;
        this.ifcAxis1Placement = (props.ifcAxis1Placement) ? new IfcAxis1Placement(props.ifcAxis1Placement) : undefined;
        this.ifcAxis2Placement2D = (props.ifcAxis2Placement2D) ? new IfcAxis2Placement2D(props.ifcAxis2Placement2D) : undefined;
        this.ifcAxis2Placement3D = (props.ifcAxis2Placement3D) ? new IfcAxis2Placement3D(props.ifcAxis2Placement3D) : undefined;
        this.ifcBSplineCurve = (props.ifcBSplineCurve) ? new IfcBSplineCurve(props.ifcBSplineCurve) : undefined;
        this.ifcBSplineCurveWithKnots = (props.ifcBSplineCurveWithKnots) ? new IfcBSplineCurveWithKnots(props.ifcBSplineCurveWithKnots) : undefined;
        this.ifcBSplineSurface = (props.ifcBSplineSurface) ? new IfcBSplineSurface(props.ifcBSplineSurface) : undefined;
        this.ifcBSplineSurfaceWithKnots = (props.ifcBSplineSurfaceWithKnots) ? new IfcBSplineSurfaceWithKnots(props.ifcBSplineSurfaceWithKnots) : undefined;
        this.ifcBeam = (props.ifcBeam) ? new IfcBeam(props.ifcBeam) : undefined;
        this.ifcBeamStandardCase = (props.ifcBeamStandardCase) ? new IfcBeamStandardCase(props.ifcBeamStandardCase) : undefined;
        this.ifcBeamType = (props.ifcBeamType) ? new IfcBeamType(props.ifcBeamType) : undefined;
        this.ifcBlobTexture = (props.ifcBlobTexture) ? new IfcBlobTexture(props.ifcBlobTexture) : undefined;
        this.ifcBlock = (props.ifcBlock) ? new IfcBlock(props.ifcBlock) : undefined;
        this.ifcBoiler = (props.ifcBoiler) ? new IfcBoiler(props.ifcBoiler) : undefined;
        this.ifcBoilerType = (props.ifcBoilerType) ? new IfcBoilerType(props.ifcBoilerType) : undefined;
        this.ifcBooleanClippingResult = (props.ifcBooleanClippingResult) ? new IfcBooleanClippingResult(props.ifcBooleanClippingResult) : undefined;
        this.ifcBooleanResult = (props.ifcBooleanResult) ? new IfcBooleanResult(props.ifcBooleanResult) : undefined;
        this.ifcBoundaryCondition = (props.ifcBoundaryCondition) ? new IfcBoundaryCondition(props.ifcBoundaryCondition) : undefined;
        this.ifcBoundaryCurve = (props.ifcBoundaryCurve) ? new IfcBoundaryCurve(props.ifcBoundaryCurve) : undefined;
        this.ifcBoundaryEdgeCondition = (props.ifcBoundaryEdgeCondition) ? new IfcBoundaryEdgeCondition(props.ifcBoundaryEdgeCondition) : undefined;
        this.ifcBoundaryFaceCondition = (props.ifcBoundaryFaceCondition) ? new IfcBoundaryFaceCondition(props.ifcBoundaryFaceCondition) : undefined;
        this.ifcBoundaryNodeCondition = (props.ifcBoundaryNodeCondition) ? new IfcBoundaryNodeCondition(props.ifcBoundaryNodeCondition) : undefined;
        this.ifcBoundaryNodeConditionWarping = (props.ifcBoundaryNodeConditionWarping) ? new IfcBoundaryNodeConditionWarping(props.ifcBoundaryNodeConditionWarping) : undefined;
        this.ifcBoundedCurve = (props.ifcBoundedCurve) ? new IfcBoundedCurve(props.ifcBoundedCurve) : undefined;
        this.ifcBoundedSurface = (props.ifcBoundedSurface) ? new IfcBoundedSurface(props.ifcBoundedSurface) : undefined;
        this.ifcBoundingBox = (props.ifcBoundingBox) ? new IfcBoundingBox(props.ifcBoundingBox) : undefined;
        this.ifcBoxedHalfSpace = (props.ifcBoxedHalfSpace) ? new IfcBoxedHalfSpace(props.ifcBoxedHalfSpace) : undefined;
        this.ifcBuilding = (props.ifcBuilding) ? new IfcBuilding(props.ifcBuilding) : undefined;
        this.ifcBuildingElement = (props.ifcBuildingElement) ? new IfcBuildingElement(props.ifcBuildingElement) : undefined;
        this.ifcBuildingElementPart = (props.ifcBuildingElementPart) ? new IfcBuildingElementPart(props.ifcBuildingElementPart) : undefined;
        this.ifcBuildingElementPartType = (props.ifcBuildingElementPartType) ? new IfcBuildingElementPartType(props.ifcBuildingElementPartType) : undefined;
        this.ifcBuildingElementProxy = (props.ifcBuildingElementProxy) ? new IfcBuildingElementProxy(props.ifcBuildingElementProxy) : undefined;
        this.ifcBuildingElementProxyType = (props.ifcBuildingElementProxyType) ? new IfcBuildingElementProxyType(props.ifcBuildingElementProxyType) : undefined;
        this.ifcBuildingElementType = (props.ifcBuildingElementType) ? new IfcBuildingElementType(props.ifcBuildingElementType) : undefined;
        this.ifcBuildingStorey = (props.ifcBuildingStorey) ? new IfcBuildingStorey(props.ifcBuildingStorey) : undefined;
        this.ifcBuildingSystem = (props.ifcBuildingSystem) ? new IfcBuildingSystem(props.ifcBuildingSystem) : undefined;
        this.ifcBurner = (props.ifcBurner) ? new IfcBurner(props.ifcBurner) : undefined;
        this.ifcBurnerType = (props.ifcBurnerType) ? new IfcBurnerType(props.ifcBurnerType) : undefined;
        this.ifcCShapeProfileDef = (props.ifcCShapeProfileDef) ? new IfcCShapeProfileDef(props.ifcCShapeProfileDef) : undefined;
        this.ifcCableCarrierFitting = (props.ifcCableCarrierFitting) ? new IfcCableCarrierFitting(props.ifcCableCarrierFitting) : undefined;
        this.ifcCableCarrierFittingType = (props.ifcCableCarrierFittingType) ? new IfcCableCarrierFittingType(props.ifcCableCarrierFittingType) : undefined;
        this.ifcCableCarrierSegment = (props.ifcCableCarrierSegment) ? new IfcCableCarrierSegment(props.ifcCableCarrierSegment) : undefined;
        this.ifcCableCarrierSegmentType = (props.ifcCableCarrierSegmentType) ? new IfcCableCarrierSegmentType(props.ifcCableCarrierSegmentType) : undefined;
        this.ifcCableFitting = (props.ifcCableFitting) ? new IfcCableFitting(props.ifcCableFitting) : undefined;
        this.ifcCableFittingType = (props.ifcCableFittingType) ? new IfcCableFittingType(props.ifcCableFittingType) : undefined;
        this.ifcCableSegment = (props.ifcCableSegment) ? new IfcCableSegment(props.ifcCableSegment) : undefined;
        this.ifcCableSegmentType = (props.ifcCableSegmentType) ? new IfcCableSegmentType(props.ifcCableSegmentType) : undefined;
        this.ifcCartesianPoint = (props.ifcCartesianPoint) ? new IfcCartesianPoint(props.ifcCartesianPoint) : undefined;
        this.ifcCartesianPointList = (props.ifcCartesianPointList) ? new IfcCartesianPointList(props.ifcCartesianPointList) : undefined;
        this.ifcCartesianPointList2D = (props.ifcCartesianPointList2D) ? new IfcCartesianPointList2D(props.ifcCartesianPointList2D) : undefined;
        this.ifcCartesianPointList3D = (props.ifcCartesianPointList3D) ? new IfcCartesianPointList3D(props.ifcCartesianPointList3D) : undefined;
        this.ifcCartesianTransformationOperator = (props.ifcCartesianTransformationOperator) ? new IfcCartesianTransformationOperator(props.ifcCartesianTransformationOperator) : undefined;
        this.ifcCartesianTransformationOperator2D = (props.ifcCartesianTransformationOperator2D) ? new IfcCartesianTransformationOperator2D(props.ifcCartesianTransformationOperator2D) : undefined;
        this.ifcCartesianTransformationOperator2DnonUniform = (props.ifcCartesianTransformationOperator2DnonUniform) ? new IfcCartesianTransformationOperator2DnonUniform(props.ifcCartesianTransformationOperator2DnonUniform) : undefined;
        this.ifcCartesianTransformationOperator3D = (props.ifcCartesianTransformationOperator3D) ? new IfcCartesianTransformationOperator3D(props.ifcCartesianTransformationOperator3D) : undefined;
        this.ifcCartesianTransformationOperator3DnonUniform = (props.ifcCartesianTransformationOperator3DnonUniform) ? new IfcCartesianTransformationOperator3DnonUniform(props.ifcCartesianTransformationOperator3DnonUniform) : undefined;
        this.ifcCenterLineProfileDef = (props.ifcCenterLineProfileDef) ? new IfcCenterLineProfileDef(props.ifcCenterLineProfileDef) : undefined;
        this.ifcChiller = (props.ifcChiller) ? new IfcChiller(props.ifcChiller) : undefined;
        this.ifcChillerType = (props.ifcChillerType) ? new IfcChillerType(props.ifcChillerType) : undefined;
        this.ifcChimney = (props.ifcChimney) ? new IfcChimney(props.ifcChimney) : undefined;
        this.ifcChimneyType = (props.ifcChimneyType) ? new IfcChimneyType(props.ifcChimneyType) : undefined;
        this.ifcCircle = (props.ifcCircle) ? new IfcCircle(props.ifcCircle) : undefined;
        this.ifcCircleHollowProfileDef = (props.ifcCircleHollowProfileDef) ? new IfcCircleHollowProfileDef(props.ifcCircleHollowProfileDef) : undefined;
        this.ifcCircleProfileDef = (props.ifcCircleProfileDef) ? new IfcCircleProfileDef(props.ifcCircleProfileDef) : undefined;
        this.ifcCivilElement = (props.ifcCivilElement) ? new IfcCivilElement(props.ifcCivilElement) : undefined;
        this.ifcCivilElementType = (props.ifcCivilElementType) ? new IfcCivilElementType(props.ifcCivilElementType) : undefined;
        this.ifcClassification = (props.ifcClassification) ? new IfcClassification(props.ifcClassification) : undefined;
        this.ifcClassificationReference = (props.ifcClassificationReference) ? new IfcClassificationReference(props.ifcClassificationReference) : undefined;
        this.ifcClosedShell = (props.ifcClosedShell) ? new IfcClosedShell(props.ifcClosedShell) : undefined;
        this.ifcCoil = (props.ifcCoil) ? new IfcCoil(props.ifcCoil) : undefined;
        this.ifcCoilType = (props.ifcCoilType) ? new IfcCoilType(props.ifcCoilType) : undefined;
        this.ifcColourRgb = (props.ifcColourRgb) ? new IfcColourRgb(props.ifcColourRgb) : undefined;
        this.ifcColourRgbList = (props.ifcColourRgbList) ? new IfcColourRgbList(props.ifcColourRgbList) : undefined;
        this.ifcColourSpecification = (props.ifcColourSpecification) ? new IfcColourSpecification(props.ifcColourSpecification) : undefined;
        this.ifcColumn = (props.ifcColumn) ? new IfcColumn(props.ifcColumn) : undefined;
        this.ifcColumnStandardCase = (props.ifcColumnStandardCase) ? new IfcColumnStandardCase(props.ifcColumnStandardCase) : undefined;
        this.ifcColumnType = (props.ifcColumnType) ? new IfcColumnType(props.ifcColumnType) : undefined;
        this.ifcCommunicationsAppliance = (props.ifcCommunicationsAppliance) ? new IfcCommunicationsAppliance(props.ifcCommunicationsAppliance) : undefined;
        this.ifcCommunicationsApplianceType = (props.ifcCommunicationsApplianceType) ? new IfcCommunicationsApplianceType(props.ifcCommunicationsApplianceType) : undefined;
        this.ifcComplexProperty = (props.ifcComplexProperty) ? new IfcComplexProperty(props.ifcComplexProperty) : undefined;
        this.ifcComplexPropertyTemplate = (props.ifcComplexPropertyTemplate) ? new IfcComplexPropertyTemplate(props.ifcComplexPropertyTemplate) : undefined;
        this.ifcCompositeCurve = props.ifcCompositeCurve;
        this.ifcCompositeCurveOnSurface = (props.ifcCompositeCurveOnSurface) ? new IfcCompositeCurveOnSurface(props.ifcCompositeCurveOnSurface) : undefined;
        this.ifcCompositeCurveSegment = (props.ifcCompositeCurveSegment) ? new IfcCompositeCurveSegment(props.ifcCompositeCurveSegment) : undefined;
        this.ifcCompositeProfileDef = props.ifcCompositeProfileDef;
        this.ifcCompressor = (props.ifcCompressor) ? new IfcCompressor(props.ifcCompressor) : undefined;
        this.ifcCompressorType = (props.ifcCompressorType) ? new IfcCompressorType(props.ifcCompressorType) : undefined;
        this.ifcCondenser = (props.ifcCondenser) ? new IfcCondenser(props.ifcCondenser) : undefined;
        this.ifcCondenserType = (props.ifcCondenserType) ? new IfcCondenserType(props.ifcCondenserType) : undefined;
        this.ifcConic = (props.ifcConic) ? new IfcConic(props.ifcConic) : undefined;
        this.ifcConnectedFaceSet = (props.ifcConnectedFaceSet) ? new IfcConnectedFaceSet(props.ifcConnectedFaceSet) : undefined;
        this.ifcConnectionCurveGeometry = (props.ifcConnectionCurveGeometry) ? new IfcConnectionCurveGeometry(props.ifcConnectionCurveGeometry) : undefined;
        this.ifcConnectionGeometry = (props.ifcConnectionGeometry) ? new IfcConnectionGeometry(props.ifcConnectionGeometry) : undefined;
        this.ifcConnectionPointEccentricity = (props.ifcConnectionPointEccentricity) ? new IfcConnectionPointEccentricity(props.ifcConnectionPointEccentricity) : undefined;
        this.ifcConnectionPointGeometry = (props.ifcConnectionPointGeometry) ? new IfcConnectionPointGeometry(props.ifcConnectionPointGeometry) : undefined;
        this.ifcConnectionSurfaceGeometry = (props.ifcConnectionSurfaceGeometry) ? new IfcConnectionSurfaceGeometry(props.ifcConnectionSurfaceGeometry) : undefined;
        this.ifcConnectionVolumeGeometry = (props.ifcConnectionVolumeGeometry) ? new IfcConnectionVolumeGeometry(props.ifcConnectionVolumeGeometry) : undefined;
        this.ifcConstraint = (props.ifcConstraint) ? new IfcConstraint(props.ifcConstraint) : undefined;
        this.ifcConstructionEquipmentResource = (props.ifcConstructionEquipmentResource) ? new IfcConstructionEquipmentResource(props.ifcConstructionEquipmentResource) : undefined;
        this.ifcConstructionEquipmentResourceType = (props.ifcConstructionEquipmentResourceType) ? new IfcConstructionEquipmentResourceType(props.ifcConstructionEquipmentResourceType) : undefined;
        this.ifcConstructionMaterialResource = (props.ifcConstructionMaterialResource) ? new IfcConstructionMaterialResource(props.ifcConstructionMaterialResource) : undefined;
        this.ifcConstructionMaterialResourceType = (props.ifcConstructionMaterialResourceType) ? new IfcConstructionMaterialResourceType(props.ifcConstructionMaterialResourceType) : undefined;
        this.ifcConstructionProductResource = (props.ifcConstructionProductResource) ? new IfcConstructionProductResource(props.ifcConstructionProductResource) : undefined;
        this.ifcConstructionProductResourceType = (props.ifcConstructionProductResourceType) ? new IfcConstructionProductResourceType(props.ifcConstructionProductResourceType) : undefined;
        this.ifcConstructionResource = (props.ifcConstructionResource) ? new IfcConstructionResource(props.ifcConstructionResource) : undefined;
        this.ifcConstructionResourceType = (props.ifcConstructionResourceType) ? new IfcConstructionResourceType(props.ifcConstructionResourceType) : undefined;
        this.ifcContext = (props.ifcContext) ? new IfcContext(props.ifcContext) : undefined;
        this.ifcContextDependentUnit = (props.ifcContextDependentUnit) ? new IfcContextDependentUnit(props.ifcContextDependentUnit) : undefined;
        this.ifcControl = (props.ifcControl) ? new IfcControl(props.ifcControl) : undefined;
        this.ifcController = (props.ifcController) ? new IfcController(props.ifcController) : undefined;
        this.ifcControllerType = (props.ifcControllerType) ? new IfcControllerType(props.ifcControllerType) : undefined;
        this.ifcConversionBasedUnit = (props.ifcConversionBasedUnit) ? new IfcConversionBasedUnit(props.ifcConversionBasedUnit) : undefined;
        this.ifcConversionBasedUnitWithOffset = (props.ifcConversionBasedUnitWithOffset) ? new IfcConversionBasedUnitWithOffset(props.ifcConversionBasedUnitWithOffset) : undefined;
        this.ifcCooledBeam = (props.ifcCooledBeam) ? new IfcCooledBeam(props.ifcCooledBeam) : undefined;
        this.ifcCooledBeamType = (props.ifcCooledBeamType) ? new IfcCooledBeamType(props.ifcCooledBeamType) : undefined;
        this.ifcCoolingTower = (props.ifcCoolingTower) ? new IfcCoolingTower(props.ifcCoolingTower) : undefined;
        this.ifcCoolingTowerType = (props.ifcCoolingTowerType) ? new IfcCoolingTowerType(props.ifcCoolingTowerType) : undefined;
        this.ifcCoordinateOperation = (props.ifcCoordinateOperation) ? new IfcCoordinateOperation(props.ifcCoordinateOperation) : undefined;
        this.ifcCoordinateReferenceSystem = (props.ifcCoordinateReferenceSystem) ? new IfcCoordinateReferenceSystem(props.ifcCoordinateReferenceSystem) : undefined;
        this.ifcCostItem = (props.ifcCostItem) ? new IfcCostItem(props.ifcCostItem) : undefined;
        this.ifcCostSchedule = (props.ifcCostSchedule) ? new IfcCostSchedule(props.ifcCostSchedule) : undefined;
        this.ifcCostValue = (props.ifcCostValue) ? new IfcCostValue(props.ifcCostValue) : undefined;
        this.ifcCovering = (props.ifcCovering) ? new IfcCovering(props.ifcCovering) : undefined;
        this.ifcCoveringType = (props.ifcCoveringType) ? new IfcCoveringType(props.ifcCoveringType) : undefined;
        this.ifcCrewResource = (props.ifcCrewResource) ? new IfcCrewResource(props.ifcCrewResource) : undefined;
        this.ifcCrewResourceType = (props.ifcCrewResourceType) ? new IfcCrewResourceType(props.ifcCrewResourceType) : undefined;
        this.ifcCsgPrimitive3D = (props.ifcCsgPrimitive3D) ? new IfcCsgPrimitive3D(props.ifcCsgPrimitive3D) : undefined;
        this.ifcCsgSolid = (props.ifcCsgSolid) ? new IfcCsgSolid(props.ifcCsgSolid) : undefined;
        this.ifcCurrencyRelationship = (props.ifcCurrencyRelationship) ? new IfcCurrencyRelationship(props.ifcCurrencyRelationship) : undefined;
        this.ifcCurtainWall = (props.ifcCurtainWall) ? new IfcCurtainWall(props.ifcCurtainWall) : undefined;
        this.ifcCurtainWallType = (props.ifcCurtainWallType) ? new IfcCurtainWallType(props.ifcCurtainWallType) : undefined;
        this.ifcCurve = (props.ifcCurve) ? new IfcCurve(props.ifcCurve) : undefined;
        this.ifcCurveBoundedPlane = (props.ifcCurveBoundedPlane) ? new IfcCurveBoundedPlane(props.ifcCurveBoundedPlane) : undefined;
        this.ifcCurveBoundedSurface = (props.ifcCurveBoundedSurface) ? new IfcCurveBoundedSurface(props.ifcCurveBoundedSurface) : undefined;
        this.ifcCurveStyle = (props.ifcCurveStyle) ? new IfcCurveStyle(props.ifcCurveStyle) : undefined;
        this.ifcCurveStyleFont = (props.ifcCurveStyleFont) ? new IfcCurveStyleFont(props.ifcCurveStyleFont) : undefined;
        this.ifcCurveStyleFontAndScaling = (props.ifcCurveStyleFontAndScaling) ? new IfcCurveStyleFontAndScaling(props.ifcCurveStyleFontAndScaling) : undefined;
        this.ifcCurveStyleFontPattern = (props.ifcCurveStyleFontPattern) ? new IfcCurveStyleFontPattern(props.ifcCurveStyleFontPattern) : undefined;
        this.ifcCylindricalSurface = (props.ifcCylindricalSurface) ? new IfcCylindricalSurface(props.ifcCylindricalSurface) : undefined;
        this.ifcDamper = (props.ifcDamper) ? new IfcDamper(props.ifcDamper) : undefined;
        this.ifcDamperType = (props.ifcDamperType) ? new IfcDamperType(props.ifcDamperType) : undefined;
        this.ifcDerivedProfileDef = (props.ifcDerivedProfileDef) ? new IfcDerivedProfileDef(props.ifcDerivedProfileDef) : undefined;
        this.ifcDerivedUnit = (props.ifcDerivedUnit) ? new IfcDerivedUnit(props.ifcDerivedUnit) : undefined;
        this.ifcDerivedUnitElement = (props.ifcDerivedUnitElement) ? new IfcDerivedUnitElement(props.ifcDerivedUnitElement) : undefined;
        this.ifcDimensionalExponents = (props.ifcDimensionalExponents) ? new IfcDimensionalExponents(props.ifcDimensionalExponents) : undefined;
        this.ifcDirection = (props.ifcDirection) ? new IfcDirection(props.ifcDirection) : undefined;
        this.ifcDiscreteAccessory = (props.ifcDiscreteAccessory) ? new IfcDiscreteAccessory(props.ifcDiscreteAccessory) : undefined;
        this.ifcDiscreteAccessoryType = (props.ifcDiscreteAccessoryType) ? new IfcDiscreteAccessoryType(props.ifcDiscreteAccessoryType) : undefined;
        this.ifcDistributionChamberElement = (props.ifcDistributionChamberElement) ? new IfcDistributionChamberElement(props.ifcDistributionChamberElement) : undefined;
        this.ifcDistributionChamberElementType = (props.ifcDistributionChamberElementType) ? new IfcDistributionChamberElementType(props.ifcDistributionChamberElementType) : undefined;
        this.ifcDistributionCircuit = (props.ifcDistributionCircuit) ? new IfcDistributionCircuit(props.ifcDistributionCircuit) : undefined;
        this.ifcDistributionControlElement = (props.ifcDistributionControlElement) ? new IfcDistributionControlElement(props.ifcDistributionControlElement) : undefined;
        this.ifcDistributionControlElementType = (props.ifcDistributionControlElementType) ? new IfcDistributionControlElementType(props.ifcDistributionControlElementType) : undefined;
        this.ifcDistributionElement = (props.ifcDistributionElement) ? new IfcDistributionElement(props.ifcDistributionElement) : undefined;
        this.ifcDistributionElementType = (props.ifcDistributionElementType) ? new IfcDistributionElementType(props.ifcDistributionElementType) : undefined;
        this.ifcDistributionFlowElement = (props.ifcDistributionFlowElement) ? new IfcDistributionFlowElement(props.ifcDistributionFlowElement) : undefined;
        this.ifcDistributionFlowElementType = (props.ifcDistributionFlowElementType) ? new IfcDistributionFlowElementType(props.ifcDistributionFlowElementType) : undefined;
        this.ifcDistributionPort = (props.ifcDistributionPort) ? new IfcDistributionPort(props.ifcDistributionPort) : undefined;
        this.ifcDistributionSystem = (props.ifcDistributionSystem) ? new IfcDistributionSystem(props.ifcDistributionSystem) : undefined;
        this.ifcDocumentInformation = (props.ifcDocumentInformation) ? new IfcDocumentInformation(props.ifcDocumentInformation) : undefined;
        this.ifcDocumentInformationRelationship = (props.ifcDocumentInformationRelationship) ? new IfcDocumentInformationRelationship(props.ifcDocumentInformationRelationship) : undefined;
        this.ifcDocumentReference = (props.ifcDocumentReference) ? new IfcDocumentReference(props.ifcDocumentReference) : undefined;
        this.ifcDoor = (props.ifcDoor) ? new IfcDoor(props.ifcDoor) : undefined;
        this.ifcDoorLiningProperties = (props.ifcDoorLiningProperties) ? new IfcDoorLiningProperties(props.ifcDoorLiningProperties) : undefined;
        this.ifcDoorPanelProperties = (props.ifcDoorPanelProperties) ? new IfcDoorPanelProperties(props.ifcDoorPanelProperties) : undefined;
        this.ifcDoorStandardCase = (props.ifcDoorStandardCase) ? new IfcDoorStandardCase(props.ifcDoorStandardCase) : undefined;
        this.ifcDoorStyle = (props.ifcDoorStyle) ? new IfcDoorStyle(props.ifcDoorStyle) : undefined;
        this.ifcDoorType = (props.ifcDoorType) ? new IfcDoorType(props.ifcDoorType) : undefined;
        this.ifcDraughtingPreDefinedColour = (props.ifcDraughtingPreDefinedColour) ? new IfcDraughtingPreDefinedColour(props.ifcDraughtingPreDefinedColour) : undefined;
        this.ifcDraughtingPreDefinedCurveFont = (props.ifcDraughtingPreDefinedCurveFont) ? new IfcDraughtingPreDefinedCurveFont(props.ifcDraughtingPreDefinedCurveFont) : undefined;
        this.ifcDuctFitting = (props.ifcDuctFitting) ? new IfcDuctFitting(props.ifcDuctFitting) : undefined;
        this.ifcDuctFittingType = (props.ifcDuctFittingType) ? new IfcDuctFittingType(props.ifcDuctFittingType) : undefined;
        this.ifcDuctSegment = (props.ifcDuctSegment) ? new IfcDuctSegment(props.ifcDuctSegment) : undefined;
        this.ifcDuctSegmentType = (props.ifcDuctSegmentType) ? new IfcDuctSegmentType(props.ifcDuctSegmentType) : undefined;
        this.ifcDuctSilencer = (props.ifcDuctSilencer) ? new IfcDuctSilencer(props.ifcDuctSilencer) : undefined;
        this.ifcDuctSilencerType = (props.ifcDuctSilencerType) ? new IfcDuctSilencerType(props.ifcDuctSilencerType) : undefined;
        this.ifcEdge = (props.ifcEdge) ? new IfcEdge(props.ifcEdge) : undefined;
        this.ifcEdgeCurve = (props.ifcEdgeCurve) ? new IfcEdgeCurve(props.ifcEdgeCurve) : undefined;
        this.ifcEdgeLoop = (props.ifcEdgeLoop) ? new IfcEdgeLoop(props.ifcEdgeLoop) : undefined;
        this.ifcElectricAppliance = (props.ifcElectricAppliance) ? new IfcElectricAppliance(props.ifcElectricAppliance) : undefined;
        this.ifcElectricApplianceType = (props.ifcElectricApplianceType) ? new IfcElectricApplianceType(props.ifcElectricApplianceType) : undefined;
        this.ifcElectricDistributionBoard = (props.ifcElectricDistributionBoard) ? new IfcElectricDistributionBoard(props.ifcElectricDistributionBoard) : undefined;
        this.ifcElectricDistributionBoardType = (props.ifcElectricDistributionBoardType) ? new IfcElectricDistributionBoardType(props.ifcElectricDistributionBoardType) : undefined;
        this.ifcElectricFlowStorageDevice = (props.ifcElectricFlowStorageDevice) ? new IfcElectricFlowStorageDevice(props.ifcElectricFlowStorageDevice) : undefined;
        this.ifcElectricFlowStorageDeviceType = (props.ifcElectricFlowStorageDeviceType) ? new IfcElectricFlowStorageDeviceType(props.ifcElectricFlowStorageDeviceType) : undefined;
        this.ifcElectricGenerator = (props.ifcElectricGenerator) ? new IfcElectricGenerator(props.ifcElectricGenerator) : undefined;
        this.ifcElectricGeneratorType = (props.ifcElectricGeneratorType) ? new IfcElectricGeneratorType(props.ifcElectricGeneratorType) : undefined;
        this.ifcElectricMotor = (props.ifcElectricMotor) ? new IfcElectricMotor(props.ifcElectricMotor) : undefined;
        this.ifcElectricMotorType = (props.ifcElectricMotorType) ? new IfcElectricMotorType(props.ifcElectricMotorType) : undefined;
        this.ifcElectricTimeControl = (props.ifcElectricTimeControl) ? new IfcElectricTimeControl(props.ifcElectricTimeControl) : undefined;
        this.ifcElectricTimeControlType = (props.ifcElectricTimeControlType) ? new IfcElectricTimeControlType(props.ifcElectricTimeControlType) : undefined;
        this.ifcElement = (props.ifcElement) ? new IfcElement(props.ifcElement) : undefined;
        this.ifcElementAssembly = (props.ifcElementAssembly) ? new IfcElementAssembly(props.ifcElementAssembly) : undefined;
        this.ifcElementAssemblyType = (props.ifcElementAssemblyType) ? new IfcElementAssemblyType(props.ifcElementAssemblyType) : undefined;
        this.ifcElementComponent = (props.ifcElementComponent) ? new IfcElementComponent(props.ifcElementComponent) : undefined;
        this.ifcElementComponentType = (props.ifcElementComponentType) ? new IfcElementComponentType(props.ifcElementComponentType) : undefined;
        this.ifcElementQuantity = (props.ifcElementQuantity) ? new IfcElementQuantity(props.ifcElementQuantity) : undefined;
        this.ifcElementType = (props.ifcElementType) ? new IfcElementType(props.ifcElementType) : undefined;
        this.ifcElementarySurface = (props.ifcElementarySurface) ? new IfcElementarySurface(props.ifcElementarySurface) : undefined;
        this.ifcEllipse = (props.ifcEllipse) ? new IfcEllipse(props.ifcEllipse) : undefined;
        this.ifcEllipseProfileDef = (props.ifcEllipseProfileDef) ? new IfcEllipseProfileDef(props.ifcEllipseProfileDef) : undefined;
        this.ifcEnergyConversionDevice = (props.ifcEnergyConversionDevice) ? new IfcEnergyConversionDevice(props.ifcEnergyConversionDevice) : undefined;
        this.ifcEnergyConversionDeviceType = (props.ifcEnergyConversionDeviceType) ? new IfcEnergyConversionDeviceType(props.ifcEnergyConversionDeviceType) : undefined;
        this.ifcEngine = (props.ifcEngine) ? new IfcEngine(props.ifcEngine) : undefined;
        this.ifcEngineType = (props.ifcEngineType) ? new IfcEngineType(props.ifcEngineType) : undefined;
        this.ifcEvaporativeCooler = (props.ifcEvaporativeCooler) ? new IfcEvaporativeCooler(props.ifcEvaporativeCooler) : undefined;
        this.ifcEvaporativeCoolerType = (props.ifcEvaporativeCoolerType) ? new IfcEvaporativeCoolerType(props.ifcEvaporativeCoolerType) : undefined;
        this.ifcEvaporator = (props.ifcEvaporator) ? new IfcEvaporator(props.ifcEvaporator) : undefined;
        this.ifcEvaporatorType = (props.ifcEvaporatorType) ? new IfcEvaporatorType(props.ifcEvaporatorType) : undefined;
        this.ifcEvent = (props.ifcEvent) ? new IfcEvent(props.ifcEvent) : undefined;
        this.ifcEventTime = (props.ifcEventTime) ? new IfcEventTime(props.ifcEventTime) : undefined;
        this.ifcEventType = (props.ifcEventType) ? new IfcEventType(props.ifcEventType) : undefined;
        this.ifcExtendedProperties = (props.ifcExtendedProperties) ? new IfcExtendedProperties(props.ifcExtendedProperties) : undefined;
        this.ifcExternalInformation = (props.ifcExternalInformation) ? new IfcExternalInformation(props.ifcExternalInformation) : undefined;
        this.ifcExternalReference = (props.ifcExternalReference) ? new IfcExternalReference(props.ifcExternalReference) : undefined;
        this.ifcExternalReferenceRelationship = (props.ifcExternalReferenceRelationship) ? new IfcExternalReferenceRelationship(props.ifcExternalReferenceRelationship) : undefined;
        this.ifcExternalSpatialElement = (props.ifcExternalSpatialElement) ? new IfcExternalSpatialElement(props.ifcExternalSpatialElement) : undefined;
        this.ifcExternalSpatialStructureElement = (props.ifcExternalSpatialStructureElement) ? new IfcExternalSpatialStructureElement(props.ifcExternalSpatialStructureElement) : undefined;
        this.ifcExternallyDefinedHatchStyle = (props.ifcExternallyDefinedHatchStyle) ? new IfcExternallyDefinedHatchStyle(props.ifcExternallyDefinedHatchStyle) : undefined;
        this.ifcExternallyDefinedSurfaceStyle = (props.ifcExternallyDefinedSurfaceStyle) ? new IfcExternallyDefinedSurfaceStyle(props.ifcExternallyDefinedSurfaceStyle) : undefined;
        this.ifcExternallyDefinedTextFont = (props.ifcExternallyDefinedTextFont) ? new IfcExternallyDefinedTextFont(props.ifcExternallyDefinedTextFont) : undefined;
        this.ifcExtrudedAreaSolid = (props.ifcExtrudedAreaSolid) ? new IfcExtrudedAreaSolid(props.ifcExtrudedAreaSolid) : undefined;
        this.ifcExtrudedAreaSolidTapered = (props.ifcExtrudedAreaSolidTapered) ? new IfcExtrudedAreaSolidTapered(props.ifcExtrudedAreaSolidTapered) : undefined;
        this.ifcFace = props.ifcFace;
        this.ifcFaceBasedSurfaceModel = (props.ifcFaceBasedSurfaceModel) ? new IfcFaceBasedSurfaceModel(props.ifcFaceBasedSurfaceModel) : undefined;
        this.ifcFaceBound = (props.ifcFaceBound) ? new IfcFaceBound(props.ifcFaceBound) : undefined;
        this.ifcFaceOuterBound = (props.ifcFaceOuterBound) ? new IfcFaceOuterBound(props.ifcFaceOuterBound) : undefined;
        this.ifcFaceSurface = (props.ifcFaceSurface) ? new IfcFaceSurface(props.ifcFaceSurface) : undefined;
        this.ifcFacetedBrep = (props.ifcFacetedBrep) ? new IfcFacetedBrep(props.ifcFacetedBrep) : undefined;
        this.ifcFacetedBrepWithVoids = (props.ifcFacetedBrepWithVoids) ? new IfcFacetedBrepWithVoids(props.ifcFacetedBrepWithVoids) : undefined;
        this.ifcFailureConnectionCondition = (props.ifcFailureConnectionCondition) ? new IfcFailureConnectionCondition(props.ifcFailureConnectionCondition) : undefined;
        this.ifcFan = (props.ifcFan) ? new IfcFan(props.ifcFan) : undefined;
        this.ifcFanType = (props.ifcFanType) ? new IfcFanType(props.ifcFanType) : undefined;
        this.ifcFastener = (props.ifcFastener) ? new IfcFastener(props.ifcFastener) : undefined;
        this.ifcFastenerType = (props.ifcFastenerType) ? new IfcFastenerType(props.ifcFastenerType) : undefined;
        this.ifcFeatureElement = (props.ifcFeatureElement) ? new IfcFeatureElement(props.ifcFeatureElement) : undefined;
        this.ifcFeatureElementAddition = (props.ifcFeatureElementAddition) ? new IfcFeatureElementAddition(props.ifcFeatureElementAddition) : undefined;
        this.ifcFeatureElementSubtraction = (props.ifcFeatureElementSubtraction) ? new IfcFeatureElementSubtraction(props.ifcFeatureElementSubtraction) : undefined;
        this.ifcFillAreaStyle = (props.ifcFillAreaStyle) ? new IfcFillAreaStyle(props.ifcFillAreaStyle) : undefined;
        this.ifcFillAreaStyleHatching = (props.ifcFillAreaStyleHatching) ? new IfcFillAreaStyleHatching(props.ifcFillAreaStyleHatching) : undefined;
        this.ifcFillAreaStyleTiles = (props.ifcFillAreaStyleTiles) ? new IfcFillAreaStyleTiles(props.ifcFillAreaStyleTiles) : undefined;
        this.ifcFilter = (props.ifcFilter) ? new IfcFilter(props.ifcFilter) : undefined;
        this.ifcFilterType = (props.ifcFilterType) ? new IfcFilterType(props.ifcFilterType) : undefined;
        this.ifcFireSuppressionTerminal = (props.ifcFireSuppressionTerminal) ? new IfcFireSuppressionTerminal(props.ifcFireSuppressionTerminal) : undefined;
        this.ifcFireSuppressionTerminalType = (props.ifcFireSuppressionTerminalType) ? new IfcFireSuppressionTerminalType(props.ifcFireSuppressionTerminalType) : undefined;
        this.ifcFixedReferenceSweptAreaSolid = (props.ifcFixedReferenceSweptAreaSolid) ? new IfcFixedReferenceSweptAreaSolid(props.ifcFixedReferenceSweptAreaSolid) : undefined;
        this.ifcFlowController = (props.ifcFlowController) ? new IfcFlowController(props.ifcFlowController) : undefined;
        this.ifcFlowControllerType = (props.ifcFlowControllerType) ? new IfcFlowControllerType(props.ifcFlowControllerType) : undefined;
        this.ifcFlowFitting = (props.ifcFlowFitting) ? new IfcFlowFitting(props.ifcFlowFitting) : undefined;
        this.ifcFlowFittingType = (props.ifcFlowFittingType) ? new IfcFlowFittingType(props.ifcFlowFittingType) : undefined;
        this.ifcFlowInstrument = (props.ifcFlowInstrument) ? new IfcFlowInstrument(props.ifcFlowInstrument) : undefined;
        this.ifcFlowInstrumentType = (props.ifcFlowInstrumentType) ? new IfcFlowInstrumentType(props.ifcFlowInstrumentType) : undefined;
        this.ifcFlowMeter = (props.ifcFlowMeter) ? new IfcFlowMeter(props.ifcFlowMeter) : undefined;
        this.ifcFlowMeterType = (props.ifcFlowMeterType) ? new IfcFlowMeterType(props.ifcFlowMeterType) : undefined;
        this.ifcFlowMovingDevice = (props.ifcFlowMovingDevice) ? new IfcFlowMovingDevice(props.ifcFlowMovingDevice) : undefined;
        this.ifcFlowMovingDeviceType = (props.ifcFlowMovingDeviceType) ? new IfcFlowMovingDeviceType(props.ifcFlowMovingDeviceType) : undefined;
        this.ifcFlowSegment = (props.ifcFlowSegment) ? new IfcFlowSegment(props.ifcFlowSegment) : undefined;
        this.ifcFlowSegmentType = (props.ifcFlowSegmentType) ? new IfcFlowSegmentType(props.ifcFlowSegmentType) : undefined;
        this.ifcFlowStorageDevice = (props.ifcFlowStorageDevice) ? new IfcFlowStorageDevice(props.ifcFlowStorageDevice) : undefined;
        this.ifcFlowStorageDeviceType = (props.ifcFlowStorageDeviceType) ? new IfcFlowStorageDeviceType(props.ifcFlowStorageDeviceType) : undefined;
        this.ifcFlowTerminal = (props.ifcFlowTerminal) ? new IfcFlowTerminal(props.ifcFlowTerminal) : undefined;
        this.ifcFlowTerminalType = (props.ifcFlowTerminalType) ? new IfcFlowTerminalType(props.ifcFlowTerminalType) : undefined;
        this.ifcFlowTreatmentDevice = (props.ifcFlowTreatmentDevice) ? new IfcFlowTreatmentDevice(props.ifcFlowTreatmentDevice) : undefined;
        this.ifcFlowTreatmentDeviceType = (props.ifcFlowTreatmentDeviceType) ? new IfcFlowTreatmentDeviceType(props.ifcFlowTreatmentDeviceType) : undefined;
        this.ifcFooting = (props.ifcFooting) ? new IfcFooting(props.ifcFooting) : undefined;
        this.ifcFootingType = (props.ifcFootingType) ? new IfcFootingType(props.ifcFootingType) : undefined;
        this.ifcFurnishingElement = (props.ifcFurnishingElement) ? new IfcFurnishingElement(props.ifcFurnishingElement) : undefined;
        this.ifcFurnishingElementType = (props.ifcFurnishingElementType) ? new IfcFurnishingElementType(props.ifcFurnishingElementType) : undefined;
        this.ifcFurniture = (props.ifcFurniture) ? new IfcFurniture(props.ifcFurniture) : undefined;
        this.ifcFurnitureType = (props.ifcFurnitureType) ? new IfcFurnitureType(props.ifcFurnitureType) : undefined;
        this.ifcGeographicElement = (props.ifcGeographicElement) ? new IfcGeographicElement(props.ifcGeographicElement) : undefined;
        this.ifcGeographicElementType = (props.ifcGeographicElementType) ? new IfcGeographicElementType(props.ifcGeographicElementType) : undefined;
        this.ifcGeometricCurveSet = (props.ifcGeometricCurveSet) ? new IfcGeometricCurveSet(props.ifcGeometricCurveSet) : undefined;
        this.ifcGeometricRepresentationContext = (props.ifcGeometricRepresentationContext) ? new IfcGeometricRepresentationContext(props.ifcGeometricRepresentationContext) : undefined;
        this.ifcGeometricRepresentationItem = (props.ifcGeometricRepresentationItem) ? new IfcGeometricRepresentationItem(props.ifcGeometricRepresentationItem) : undefined;
        this.ifcGeometricRepresentationSubContext = (props.ifcGeometricRepresentationSubContext) ? new IfcGeometricRepresentationSubContext(props.ifcGeometricRepresentationSubContext) : undefined;
        this.ifcGeometricSet = (props.ifcGeometricSet) ? new IfcGeometricSet(props.ifcGeometricSet) : undefined;
        this.ifcGrid = (props.ifcGrid) ? new IfcGrid(props.ifcGrid) : undefined;
        this.ifcGridAxis = (props.ifcGridAxis) ? new IfcGridAxis(props.ifcGridAxis) : undefined;
        this.ifcGridPlacement = (props.ifcGridPlacement) ? new IfcGridPlacement(props.ifcGridPlacement) : undefined;
        this.ifcGroup = (props.ifcGroup) ? new IfcGroup(props.ifcGroup) : undefined;
        this.ifcHalfSpaceSolid = (props.ifcHalfSpaceSolid) ? new IfcHalfSpaceSolid(props.ifcHalfSpaceSolid) : undefined;
        this.ifcHeatExchanger = (props.ifcHeatExchanger) ? new IfcHeatExchanger(props.ifcHeatExchanger) : undefined;
        this.ifcHeatExchangerType = (props.ifcHeatExchangerType) ? new IfcHeatExchangerType(props.ifcHeatExchangerType) : undefined;
        this.ifcHumidifier = (props.ifcHumidifier) ? new IfcHumidifier(props.ifcHumidifier) : undefined;
        this.ifcHumidifierType = (props.ifcHumidifierType) ? new IfcHumidifierType(props.ifcHumidifierType) : undefined;
        this.ifcIShapeProfileDef = (props.ifcIShapeProfileDef) ? new IfcIShapeProfileDef(props.ifcIShapeProfileDef) : undefined;
        this.ifcImageTexture = (props.ifcImageTexture) ? new IfcImageTexture(props.ifcImageTexture) : undefined;
        this.ifcIndexedColourMap = (props.ifcIndexedColourMap) ? new IfcIndexedColourMap(props.ifcIndexedColourMap) : undefined;
        this.ifcIndexedPolyCurve = (props.ifcIndexedPolyCurve) ? new IfcIndexedPolyCurve(props.ifcIndexedPolyCurve) : undefined;
        this.ifcIndexedPolygonalFace = (props.ifcIndexedPolygonalFace) ? new IfcIndexedPolygonalFace(props.ifcIndexedPolygonalFace) : undefined;
        this.ifcIndexedPolygonalFaceWithVoids = (_a = props.ifcIndexedPolygonalFaceWithVoids) === null || _a === void 0 ? void 0 : _a.map(function (o) { return o; });
        this.ifcIndexedTextureMap = (props.ifcIndexedTextureMap) ? new IfcIndexedTextureMap(props.ifcIndexedTextureMap) : undefined;
        this.ifcIndexedTriangleTextureMap = (props.ifcIndexedTriangleTextureMap) ? new IfcIndexedTriangleTextureMap(props.ifcIndexedTriangleTextureMap) : undefined;
        this.ifcInterceptor = (props.ifcInterceptor) ? new IfcInterceptor(props.ifcInterceptor) : undefined;
        this.ifcInterceptorType = (props.ifcInterceptorType) ? new IfcInterceptorType(props.ifcInterceptorType) : undefined;
        this.ifcIntersectionCurve = (props.ifcIntersectionCurve) ? new IfcIntersectionCurve(props.ifcIntersectionCurve) : undefined;
        this.ifcInventory = (props.ifcInventory) ? new IfcInventory(props.ifcInventory) : undefined;
        this.ifcIrregularTimeSeries = (props.ifcIrregularTimeSeries) ? new IfcIrregularTimeSeries(props.ifcIrregularTimeSeries) : undefined;
        this.ifcIrregularTimeSeriesValue = (props.ifcIrregularTimeSeriesValue) ? new IfcIrregularTimeSeriesValue(props.ifcIrregularTimeSeriesValue) : undefined;
        this.ifcJunctionBox = (props.ifcJunctionBox) ? new IfcJunctionBox(props.ifcJunctionBox) : undefined;
        this.ifcJunctionBoxType = (props.ifcJunctionBoxType) ? new IfcJunctionBoxType(props.ifcJunctionBoxType) : undefined;
        this.ifcLShapeProfileDef = (props.ifcLShapeProfileDef) ? new IfcLShapeProfileDef(props.ifcLShapeProfileDef) : undefined;
        this.ifcLaborResource = (props.ifcLaborResource) ? new IfcLaborResource(props.ifcLaborResource) : undefined;
        this.ifcLaborResourceType = (props.ifcLaborResourceType) ? new IfcLaborResourceType(props.ifcLaborResourceType) : undefined;
        this.ifcLagTime = (props.ifcLagTime) ? new IfcLagTime(props.ifcLagTime) : undefined;
        this.ifcLamp = (props.ifcLamp) ? new IfcLamp(props.ifcLamp) : undefined;
        this.ifcLampType = (props.ifcLampType) ? new IfcLampType(props.ifcLampType) : undefined;
        this.ifcLibraryInformation = (props.ifcLibraryInformation) ? new IfcLibraryInformation(props.ifcLibraryInformation) : undefined;
        this.ifcLibraryReference = (props.ifcLibraryReference) ? new IfcLibraryReference(props.ifcLibraryReference) : undefined;
        this.ifcLightDistributionData = (props.ifcLightDistributionData) ? new IfcLightDistributionData(props.ifcLightDistributionData) : undefined;
        this.ifcLightFixture = (props.ifcLightFixture) ? new IfcLightFixture(props.ifcLightFixture) : undefined;
        this.ifcLightFixtureType = (props.ifcLightFixtureType) ? new IfcLightFixtureType(props.ifcLightFixtureType) : undefined;
        this.ifcLightIntensityDistribution = (props.ifcLightIntensityDistribution) ? new IfcLightIntensityDistribution(props.ifcLightIntensityDistribution) : undefined;
        this.ifcLightSource = (props.ifcLightSource) ? new IfcLightSource(props.ifcLightSource) : undefined;
        this.ifcLightSourceAmbient = (props.ifcLightSourceAmbient) ? new IfcLightSourceAmbient(props.ifcLightSourceAmbient) : undefined;
        this.ifcLightSourceDirectional = (props.ifcLightSourceDirectional) ? new IfcLightSourceDirectional(props.ifcLightSourceDirectional) : undefined;
        this.ifcLightSourceGoniometric = (props.ifcLightSourceGoniometric) ? new IfcLightSourceGoniometric(props.ifcLightSourceGoniometric) : undefined;
        this.ifcLightSourcePositional = (props.ifcLightSourcePositional) ? new IfcLightSourcePositional(props.ifcLightSourcePositional) : undefined;
        this.ifcLightSourceSpot = (props.ifcLightSourceSpot) ? new IfcLightSourceSpot(props.ifcLightSourceSpot) : undefined;
        this.ifcLine = (props.ifcLine) ? new IfcLine(props.ifcLine) : undefined;
        this.ifcLocalPlacement = (props.ifcLocalPlacement) ? new IfcLocalPlacement(props.ifcLocalPlacement) : undefined;
        this.ifcLoop = (props.ifcLoop) ? new IfcLoop(props.ifcLoop) : undefined;
        this.ifcManifoldSolidBrep = (props.ifcManifoldSolidBrep) ? new IfcManifoldSolidBrep(props.ifcManifoldSolidBrep) : undefined;
        this.ifcMapConversion = (props.ifcMapConversion) ? new IfcMapConversion(props.ifcMapConversion) : undefined;
        this.ifcMappedItem = (props.ifcMappedItem) ? new IfcMappedItem(props.ifcMappedItem) : undefined;
        this.ifcMaterial = (props.ifcMaterial) ? new IfcMaterial(props.ifcMaterial) : undefined;
        this.ifcMaterialClassificationRelationship = (props.ifcMaterialClassificationRelationship) ? new IfcMaterialClassificationRelationship(props.ifcMaterialClassificationRelationship) : undefined;
        this.ifcMaterialConstituent = (props.ifcMaterialConstituent) ? new IfcMaterialConstituent(props.ifcMaterialConstituent) : undefined;
        this.ifcMaterialConstituentSet = (props.ifcMaterialConstituentSet) ? new IfcMaterialConstituentSet(props.ifcMaterialConstituentSet) : undefined;
        this.ifcMaterialDefinition = (props.ifcMaterialDefinition) ? new IfcMaterialDefinition(props.ifcMaterialDefinition) : undefined;
        this.ifcMaterialDefinitionRepresentation = (props.ifcMaterialDefinitionRepresentation) ? new IfcMaterialDefinitionRepresentation(props.ifcMaterialDefinitionRepresentation) : undefined;
        this.ifcMaterialLayer = (props.ifcMaterialLayer) ? new IfcMaterialLayer(props.ifcMaterialLayer) : undefined;
        this.ifcMaterialLayerSet = (props.ifcMaterialLayerSet) ? new IfcMaterialLayerSet(props.ifcMaterialLayerSet) : undefined;
        this.ifcMaterialLayerSetUsage = (props.ifcMaterialLayerSetUsage) ? new IfcMaterialLayerSetUsage(props.ifcMaterialLayerSetUsage) : undefined;
        this.ifcMaterialLayerWithOffsets = (props.ifcMaterialLayerWithOffsets) ? new IfcMaterialLayerWithOffsets(props.ifcMaterialLayerWithOffsets) : undefined;
        this.ifcMaterialList = (props.ifcMaterialList) ? new IfcMaterialList(props.ifcMaterialList) : undefined;
        this.ifcMaterialProfile = (props.ifcMaterialProfile) ? new IfcMaterialProfile(props.ifcMaterialProfile) : undefined;
        this.ifcMaterialProfileSet = (props.ifcMaterialProfileSet) ? new IfcMaterialProfileSet(props.ifcMaterialProfileSet) : undefined;
        this.ifcMaterialProfileSetUsage = (props.ifcMaterialProfileSetUsage) ? new IfcMaterialProfileSetUsage(props.ifcMaterialProfileSetUsage) : undefined;
        this.ifcMaterialProfileSetUsageTapering = (props.ifcMaterialProfileSetUsageTapering) ? new IfcMaterialProfileSetUsageTapering(props.ifcMaterialProfileSetUsageTapering) : undefined;
        this.ifcMaterialProfileWithOffsets = (props.ifcMaterialProfileWithOffsets) ? new IfcMaterialProfileWithOffsets(props.ifcMaterialProfileWithOffsets) : undefined;
        this.ifcMaterialProperties = (props.ifcMaterialProperties) ? new IfcMaterialProperties(props.ifcMaterialProperties) : undefined;
        this.ifcMaterialRelationship = (props.ifcMaterialRelationship) ? new IfcMaterialRelationship(props.ifcMaterialRelationship) : undefined;
        this.ifcMaterialUsageDefinition = (props.ifcMaterialUsageDefinition) ? new IfcMaterialUsageDefinition(props.ifcMaterialUsageDefinition) : undefined;
        this.ifcMeasureWithUnit = (props.ifcMeasureWithUnit) ? new IfcMeasureWithUnit(props.ifcMeasureWithUnit) : undefined;
        this.ifcMechanicalFastener = (props.ifcMechanicalFastener) ? new IfcMechanicalFastener(props.ifcMechanicalFastener) : undefined;
        this.ifcMechanicalFastenerType = (props.ifcMechanicalFastenerType) ? new IfcMechanicalFastenerType(props.ifcMechanicalFastenerType) : undefined;
        this.ifcMedicalDevice = (props.ifcMedicalDevice) ? new IfcMedicalDevice(props.ifcMedicalDevice) : undefined;
        this.ifcMedicalDeviceType = (props.ifcMedicalDeviceType) ? new IfcMedicalDeviceType(props.ifcMedicalDeviceType) : undefined;
        this.ifcMember = (props.ifcMember) ? new IfcMember(props.ifcMember) : undefined;
        this.ifcMemberStandardCase = (props.ifcMemberStandardCase) ? new IfcMemberStandardCase(props.ifcMemberStandardCase) : undefined;
        this.ifcMemberType = (props.ifcMemberType) ? new IfcMemberType(props.ifcMemberType) : undefined;
        this.ifcMetric = (props.ifcMetric) ? new IfcMetric(props.ifcMetric) : undefined;
        this.ifcMirroredProfileDef = (props.ifcMirroredProfileDef) ? new IfcMirroredProfileDef(props.ifcMirroredProfileDef) : undefined;
        this.ifcMonetaryUnit = (props.ifcMonetaryUnit) ? new IfcMonetaryUnit(props.ifcMonetaryUnit) : undefined;
        this.ifcMotorConnection = (props.ifcMotorConnection) ? new IfcMotorConnection(props.ifcMotorConnection) : undefined;
        this.ifcMotorConnectionType = (props.ifcMotorConnectionType) ? new IfcMotorConnectionType(props.ifcMotorConnectionType) : undefined;
        this.ifcNamedUnit = (props.ifcNamedUnit) ? new IfcNamedUnit(props.ifcNamedUnit) : undefined;
        this.ifcObject = (props.ifcObject) ? new IfcObject(props.ifcObject) : undefined;
        this.ifcObjectDefinition = (props.ifcObjectDefinition) ? new IfcObjectDefinition(props.ifcObjectDefinition) : undefined;
        this.ifcObjectPlacement = (props.ifcObjectPlacement) ? new IfcObjectPlacement(props.ifcObjectPlacement) : undefined;
        this.ifcObjective = (props.ifcObjective) ? new IfcObjective(props.ifcObjective) : undefined;
        this.ifcOccupant = (props.ifcOccupant) ? new IfcOccupant(props.ifcOccupant) : undefined;
        this.ifcOffsetCurve2D = (props.ifcOffsetCurve2D) ? new IfcOffsetCurve2D(props.ifcOffsetCurve2D) : undefined;
        this.ifcOffsetCurve3D = (props.ifcOffsetCurve3D) ? new IfcOffsetCurve3D(props.ifcOffsetCurve3D) : undefined;
        this.ifcOpenShell = (props.ifcOpenShell) ? new IfcOpenShell(props.ifcOpenShell) : undefined;
        this.ifcOpeningElement = (props.ifcOpeningElement) ? new IfcOpeningElement(props.ifcOpeningElement) : undefined;
        this.ifcOpeningStandardCase = (props.ifcOpeningStandardCase) ? new IfcOpeningStandardCase(props.ifcOpeningStandardCase) : undefined;
        this.ifcOrganization = (props.ifcOrganization) ? new IfcOrganization(props.ifcOrganization) : undefined;
        this.ifcOrganizationRelationship = (props.ifcOrganizationRelationship) ? new IfcOrganizationRelationship(props.ifcOrganizationRelationship) : undefined;
        this.ifcOrientedEdge = (props.ifcOrientedEdge) ? new IfcOrientedEdge(props.ifcOrientedEdge) : undefined;
        this.ifcOuterBoundaryCurve = (props.ifcOuterBoundaryCurve) ? new IfcOuterBoundaryCurve(props.ifcOuterBoundaryCurve) : undefined;
        this.ifcOutlet = (props.ifcOutlet) ? new IfcOutlet(props.ifcOutlet) : undefined;
        this.ifcOutletType = (props.ifcOutletType) ? new IfcOutletType(props.ifcOutletType) : undefined;
        this.ifcOwnerHistory = (props.ifcOwnerHistory) ? new IfcOwnerHistory(props.ifcOwnerHistory) : undefined;
        this.ifcParameterizedProfileDef = (props.ifcParameterizedProfileDef) ? new IfcParameterizedProfileDef(props.ifcParameterizedProfileDef) : undefined;
        this.ifcPath = (props.ifcPath) ? new IfcPath(props.ifcPath) : undefined;
        this.ifcPcurve = (props.ifcPcurve) ? new IfcPcurve(props.ifcPcurve) : undefined;
        this.ifcPerformanceHistory = (props.ifcPerformanceHistory) ? new IfcPerformanceHistory(props.ifcPerformanceHistory) : undefined;
        this.ifcPermeableCoveringProperties = (props.ifcPermeableCoveringProperties) ? new IfcPermeableCoveringProperties(props.ifcPermeableCoveringProperties) : undefined;
        this.ifcPermit = (props.ifcPermit) ? new IfcPermit(props.ifcPermit) : undefined;
        this.ifcPerson = (props.ifcPerson) ? new IfcPerson(props.ifcPerson) : undefined;
        this.ifcPersonAndOrganization = (props.ifcPersonAndOrganization) ? new IfcPersonAndOrganization(props.ifcPersonAndOrganization) : undefined;
        this.ifcPhysicalComplexQuantity = (props.ifcPhysicalComplexQuantity) ? new IfcPhysicalComplexQuantity(props.ifcPhysicalComplexQuantity) : undefined;
        this.ifcPhysicalQuantity = (props.ifcPhysicalQuantity) ? new IfcPhysicalQuantity(props.ifcPhysicalQuantity) : undefined;
        this.ifcPhysicalSimpleQuantity = (props.ifcPhysicalSimpleQuantity) ? new IfcPhysicalSimpleQuantity(props.ifcPhysicalSimpleQuantity) : undefined;
        this.ifcPile = (props.ifcPile) ? new IfcPile(props.ifcPile) : undefined;
        this.ifcPileType = (props.ifcPileType) ? new IfcPileType(props.ifcPileType) : undefined;
        this.ifcPipeFitting = (props.ifcPipeFitting) ? new IfcPipeFitting(props.ifcPipeFitting) : undefined;
        this.ifcPipeFittingType = (props.ifcPipeFittingType) ? new IfcPipeFittingType(props.ifcPipeFittingType) : undefined;
        this.ifcPipeSegment = (props.ifcPipeSegment) ? new IfcPipeSegment(props.ifcPipeSegment) : undefined;
        this.ifcPipeSegmentType = (props.ifcPipeSegmentType) ? new IfcPipeSegmentType(props.ifcPipeSegmentType) : undefined;
        this.ifcPixelTexture = (props.ifcPixelTexture) ? new IfcPixelTexture(props.ifcPixelTexture) : undefined;
        this.ifcPlacement = (props.ifcPlacement) ? new IfcPlacement(props.ifcPlacement) : undefined;
        this.ifcPlanarBox = (props.ifcPlanarBox) ? new IfcPlanarBox(props.ifcPlanarBox) : undefined;
        this.ifcPlanarExtent = (props.ifcPlanarExtent) ? new IfcPlanarExtent(props.ifcPlanarExtent) : undefined;
        this.ifcPlane = (props.ifcPlane) ? new IfcPlane(props.ifcPlane) : undefined;
        this.ifcPlate = (props.ifcPlate) ? new IfcPlate(props.ifcPlate) : undefined;
        this.ifcPlateStandardCase = (props.ifcPlateStandardCase) ? new IfcPlateStandardCase(props.ifcPlateStandardCase) : undefined;
        this.ifcPlateType = (props.ifcPlateType) ? new IfcPlateType(props.ifcPlateType) : undefined;
        this.ifcPoint = (props.ifcPoint) ? new IfcPoint(props.ifcPoint) : undefined;
        this.ifcPointOnCurve = (props.ifcPointOnCurve) ? new IfcPointOnCurve(props.ifcPointOnCurve) : undefined;
        this.ifcPointOnSurface = (props.ifcPointOnSurface) ? new IfcPointOnSurface(props.ifcPointOnSurface) : undefined;
        this.ifcPolyLoop = (props.ifcPolyLoop) ? new IfcPolyLoop(props.ifcPolyLoop) : undefined;
        this.ifcPolygonalBoundedHalfSpace = (props.ifcPolygonalBoundedHalfSpace) ? new IfcPolygonalBoundedHalfSpace(props.ifcPolygonalBoundedHalfSpace) : undefined;
        this.ifcPolygonalFaceSet = (props.ifcPolygonalFaceSet) ? new IfcPolygonalFaceSet(props.ifcPolygonalFaceSet) : undefined;
        this.ifcPolyline = (props.ifcPolyline) ? new IfcPolyline(props.ifcPolyline) : undefined;
        this.ifcPort = (props.ifcPort) ? new IfcPort(props.ifcPort) : undefined;
        this.ifcPostalAddress = props.ifcPostalAddress;
        this.ifcPreDefinedColour = (props.ifcPreDefinedColour) ? new IfcPreDefinedColour(props.ifcPreDefinedColour) : undefined;
        this.ifcPreDefinedCurveFont = (props.ifcPreDefinedCurveFont) ? new IfcPreDefinedCurveFont(props.ifcPreDefinedCurveFont) : undefined;
        this.ifcPreDefinedItem = (props.ifcPreDefinedItem) ? new IfcPreDefinedItem(props.ifcPreDefinedItem) : undefined;
        this.ifcPreDefinedProperties = (props.ifcPreDefinedProperties) ? new IfcPreDefinedProperties(props.ifcPreDefinedProperties) : undefined;
        this.ifcPreDefinedPropertySet = (props.ifcPreDefinedPropertySet) ? new IfcPreDefinedPropertySet(props.ifcPreDefinedPropertySet) : undefined;
        this.ifcPreDefinedTextFont = (props.ifcPreDefinedTextFont) ? new IfcPreDefinedTextFont(props.ifcPreDefinedTextFont) : undefined;
        this.ifcPresentationItem = (props.ifcPresentationItem) ? new IfcPresentationItem(props.ifcPresentationItem) : undefined;
        this.ifcPresentationLayerAssignment = (props.ifcPresentationLayerAssignment) ? new IfcPresentationLayerAssignment(props.ifcPresentationLayerAssignment) : undefined;
        this.ifcPresentationLayerWithStyle = (props.ifcPresentationLayerWithStyle) ? new IfcPresentationLayerWithStyle(props.ifcPresentationLayerWithStyle) : undefined;
        this.ifcPresentationStyle = (props.ifcPresentationStyle) ? new IfcPresentationStyle(props.ifcPresentationStyle) : undefined;
        this.ifcPresentationStyleAssignment = (props.ifcPresentationStyleAssignment) ? new IfcPresentationStyleAssignment(props.ifcPresentationStyleAssignment) : undefined;
        this.ifcProcedure = (props.ifcProcedure) ? new IfcProcedure(props.ifcProcedure) : undefined;
        this.ifcProcedureType = (props.ifcProcedureType) ? new IfcProcedureType(props.ifcProcedureType) : undefined;
        this.ifcProcess = (props.ifcProcess) ? new IfcProcess(props.ifcProcess) : undefined;
        this.ifcProduct = (props.ifcProduct) ? new IfcProduct(props.ifcProduct) : undefined;
        this.ifcProductDefinitionShape = (props.ifcProductDefinitionShape) ? new IfcProductDefinitionShape(props.ifcProductDefinitionShape) : undefined;
        this.ifcProductRepresentation = props.ifcProductRepresentation;
        this.ifcProfileDef = props.ifcProfileDef;
        this.ifcProfileProperties = (props.ifcProfileProperties) ? new IfcProfileProperties(props.ifcProfileProperties) : undefined;
        this.ifcProject = (props.ifcProject) ? new IfcProject(props.ifcProject) : undefined;
        this.ifcProjectLibrary = (props.ifcProjectLibrary) ? new IfcProjectLibrary(props.ifcProjectLibrary) : undefined;
        this.ifcProjectOrder = (props.ifcProjectOrder) ? new IfcProjectOrder(props.ifcProjectOrder) : undefined;
        this.ifcProjectedCRS = (props.ifcProjectedCRS) ? new IfcProjectedCRS(props.ifcProjectedCRS) : undefined;
        this.ifcProjectionElement = (props.ifcProjectionElement) ? new IfcProjectionElement(props.ifcProjectionElement) : undefined;
        this.ifcProperty = (props.ifcProperty) ? new IfcProperty(props.ifcProperty) : undefined;
        this.ifcPropertyAbstraction = (props.ifcPropertyAbstraction) ? new IfcPropertyAbstraction(props.ifcPropertyAbstraction) : undefined;
        this.ifcPropertyBoundedValue = (props.ifcPropertyBoundedValue) ? new IfcPropertyBoundedValue(props.ifcPropertyBoundedValue) : undefined;
        this.ifcPropertyDefinition = (props.ifcPropertyDefinition) ? new IfcPropertyDefinition(props.ifcPropertyDefinition) : undefined;
        this.ifcPropertyDependencyRelationship = (props.ifcPropertyDependencyRelationship) ? new IfcPropertyDependencyRelationship(props.ifcPropertyDependencyRelationship) : undefined;
        this.ifcPropertyEnumeratedValue = (props.ifcPropertyEnumeratedValue) ? new IfcPropertyEnumeratedValue(props.ifcPropertyEnumeratedValue) : undefined;
        this.ifcPropertyEnumeration = (props.ifcPropertyEnumeration) ? new IfcPropertyEnumeration(props.ifcPropertyEnumeration) : undefined;
        this.ifcPropertyListValue = (props.ifcPropertyListValue) ? new IfcPropertyListValue(props.ifcPropertyListValue) : undefined;
        this.ifcPropertyReferenceValue = (props.ifcPropertyReferenceValue) ? new IfcPropertyReferenceValue(props.ifcPropertyReferenceValue) : undefined;
        this.ifcPropertySet = (props.ifcPropertySet) ? new IfcPropertySet(props.ifcPropertySet) : undefined;
        this.ifcPropertySetDefinition = (props.ifcPropertySetDefinition) ? new IfcPropertySetDefinition(props.ifcPropertySetDefinition) : undefined;
        this.ifcPropertySetTemplate = props.ifcPropertySetTemplate;
        this.ifcPropertySingleValue = (props.ifcPropertySingleValue) ? new IfcPropertySingleValue(props.ifcPropertySingleValue) : undefined;
        this.ifcPropertyTableValue = (props.ifcPropertyTableValue) ? new IfcPropertyTableValue(props.ifcPropertyTableValue) : undefined;
        this.ifcPropertyTemplate = (props.ifcPropertyTemplate) ? new IfcPropertyTemplate(props.ifcPropertyTemplate) : undefined;
        this.ifcPropertyTemplateDefinition = (props.ifcPropertyTemplateDefinition) ? new IfcPropertyTemplateDefinition(props.ifcPropertyTemplateDefinition) : undefined;
        this.ifcProtectiveDevice = (props.ifcProtectiveDevice) ? new IfcProtectiveDevice(props.ifcProtectiveDevice) : undefined;
        this.ifcProtectiveDeviceTrippingUnit = (props.ifcProtectiveDeviceTrippingUnit) ? new IfcProtectiveDeviceTrippingUnit(props.ifcProtectiveDeviceTrippingUnit) : undefined;
        this.ifcProtectiveDeviceTrippingUnitType = (props.ifcProtectiveDeviceTrippingUnitType) ? new IfcProtectiveDeviceTrippingUnitType(props.ifcProtectiveDeviceTrippingUnitType) : undefined;
        this.ifcProtectiveDeviceType = (props.ifcProtectiveDeviceType) ? new IfcProtectiveDeviceType(props.ifcProtectiveDeviceType) : undefined;
        this.ifcProxy = (props.ifcProxy) ? new IfcProxy(props.ifcProxy) : undefined;
        this.ifcPump = (props.ifcPump) ? new IfcPump(props.ifcPump) : undefined;
        this.ifcPumpType = (props.ifcPumpType) ? new IfcPumpType(props.ifcPumpType) : undefined;
        this.ifcQuantityArea = (props.ifcQuantityArea) ? new IfcQuantityArea(props.ifcQuantityArea) : undefined;
        this.ifcQuantityCount = (props.ifcQuantityCount) ? new IfcQuantityCount(props.ifcQuantityCount) : undefined;
        this.ifcQuantityLength = (props.ifcQuantityLength) ? new IfcQuantityLength(props.ifcQuantityLength) : undefined;
        this.ifcQuantitySet = (props.ifcQuantitySet) ? new IfcQuantitySet(props.ifcQuantitySet) : undefined;
        this.ifcQuantityTime = (props.ifcQuantityTime) ? new IfcQuantityTime(props.ifcQuantityTime) : undefined;
        this.ifcQuantityVolume = (props.ifcQuantityVolume) ? new IfcQuantityVolume(props.ifcQuantityVolume) : undefined;
        this.ifcQuantityWeight = (props.ifcQuantityWeight) ? new IfcQuantityWeight(props.ifcQuantityWeight) : undefined;
        this.ifcRailing = (props.ifcRailing) ? new IfcRailing(props.ifcRailing) : undefined;
        this.ifcRailingType = (props.ifcRailingType) ? new IfcRailingType(props.ifcRailingType) : undefined;
        this.ifcRamp = (props.ifcRamp) ? new IfcRamp(props.ifcRamp) : undefined;
        this.ifcRampFlight = (props.ifcRampFlight) ? new IfcRampFlight(props.ifcRampFlight) : undefined;
        this.ifcRampFlightType = (props.ifcRampFlightType) ? new IfcRampFlightType(props.ifcRampFlightType) : undefined;
        this.ifcRampType = (props.ifcRampType) ? new IfcRampType(props.ifcRampType) : undefined;
        this.ifcRationalBSplineCurveWithKnots = (props.ifcRationalBSplineCurveWithKnots) ? new IfcRationalBSplineCurveWithKnots(props.ifcRationalBSplineCurveWithKnots) : undefined;
        this.ifcRationalBSplineSurfaceWithKnots = (props.ifcRationalBSplineSurfaceWithKnots) ? new IfcRationalBSplineSurfaceWithKnots(props.ifcRationalBSplineSurfaceWithKnots) : undefined;
        this.ifcRectangleHollowProfileDef = (props.ifcRectangleHollowProfileDef) ? new IfcRectangleHollowProfileDef(props.ifcRectangleHollowProfileDef) : undefined;
        this.ifcRectangleProfileDef = (props.ifcRectangleProfileDef) ? new IfcRectangleProfileDef(props.ifcRectangleProfileDef) : undefined;
        this.ifcRectangularPyramid = (props.ifcRectangularPyramid) ? new IfcRectangularPyramid(props.ifcRectangularPyramid) : undefined;
        this.ifcRectangularTrimmedSurface = (props.ifcRectangularTrimmedSurface) ? new IfcRectangularTrimmedSurface(props.ifcRectangularTrimmedSurface) : undefined;
        this.ifcRecurrencePattern = props.ifcRecurrencePattern;
        this.ifcReference = (props.ifcReference) ? new IfcReference(props.ifcReference) : undefined;
        this.ifcRegularTimeSeries = (props.ifcRegularTimeSeries) ? new IfcRegularTimeSeries(props.ifcRegularTimeSeries) : undefined;
        this.ifcReinforcementBarProperties = (props.ifcReinforcementBarProperties) ? new IfcReinforcementBarProperties(props.ifcReinforcementBarProperties) : undefined;
        this.ifcReinforcementDefinitionProperties = (props.ifcReinforcementDefinitionProperties) ? new IfcReinforcementDefinitionProperties(props.ifcReinforcementDefinitionProperties) : undefined;
        this.ifcReinforcingBar = (props.ifcReinforcingBar) ? new IfcReinforcingBar(props.ifcReinforcingBar) : undefined;
        this.ifcReinforcingBarType = (props.ifcReinforcingBarType) ? new IfcReinforcingBarType(props.ifcReinforcingBarType) : undefined;
        this.ifcReinforcingElement = (props.ifcReinforcingElement) ? new IfcReinforcingElement(props.ifcReinforcingElement) : undefined;
        this.ifcReinforcingElementType = (props.ifcReinforcingElementType) ? new IfcReinforcingElementType(props.ifcReinforcingElementType) : undefined;
        this.ifcReinforcingMesh = (props.ifcReinforcingMesh) ? new IfcReinforcingMesh(props.ifcReinforcingMesh) : undefined;
        this.ifcReinforcingMeshType = (props.ifcReinforcingMeshType) ? new IfcReinforcingMeshType(props.ifcReinforcingMeshType) : undefined;
        this.ifcRelAggregates = (props.ifcRelAggregates) ? new IfcRelAggregates(props.ifcRelAggregates) : undefined;
        this.ifcRelAssigns = (props.ifcRelAssigns) ? new IfcRelAssigns(props.ifcRelAssigns) : undefined;
        this.ifcRelAssignsToActor = (props.ifcRelAssignsToActor) ? new IfcRelAssignsToActor(props.ifcRelAssignsToActor) : undefined;
        this.ifcRelAssignsToControl = (props.ifcRelAssignsToControl) ? new IfcRelAssignsToControl(props.ifcRelAssignsToControl) : undefined;
        this.ifcRelAssignsToGroup = (props.ifcRelAssignsToGroup) ? new IfcRelAssignsToGroup(props.ifcRelAssignsToGroup) : undefined;
        this.ifcRelAssignsToGroupByFactor = (props.ifcRelAssignsToGroupByFactor) ? new IfcRelAssignsToGroupByFactor(props.ifcRelAssignsToGroupByFactor) : undefined;
        this.ifcRelAssignsToProcess = (props.ifcRelAssignsToProcess) ? new IfcRelAssignsToProcess(props.ifcRelAssignsToProcess) : undefined;
        this.ifcRelAssignsToProduct = (props.ifcRelAssignsToProduct) ? new IfcRelAssignsToProduct(props.ifcRelAssignsToProduct) : undefined;
        this.ifcRelAssignsToResource = (props.ifcRelAssignsToResource) ? new IfcRelAssignsToResource(props.ifcRelAssignsToResource) : undefined;
        this.ifcRelAssociates = (props.ifcRelAssociates) ? new IfcRelAssociates(props.ifcRelAssociates) : undefined;
        this.ifcRelAssociatesApproval = (props.ifcRelAssociatesApproval) ? new IfcRelAssociatesApproval(props.ifcRelAssociatesApproval) : undefined;
        this.ifcRelAssociatesClassification = (props.ifcRelAssociatesClassification) ? new IfcRelAssociatesClassification(props.ifcRelAssociatesClassification) : undefined;
        this.ifcRelAssociatesConstraint = (props.ifcRelAssociatesConstraint) ? new IfcRelAssociatesConstraint(props.ifcRelAssociatesConstraint) : undefined;
        this.ifcRelAssociatesDocument = (props.ifcRelAssociatesDocument) ? new IfcRelAssociatesDocument(props.ifcRelAssociatesDocument) : undefined;
        this.ifcRelAssociatesLibrary = (props.ifcRelAssociatesLibrary) ? new IfcRelAssociatesLibrary(props.ifcRelAssociatesLibrary) : undefined;
        this.ifcRelAssociatesMaterial = (props.ifcRelAssociatesMaterial) ? new IfcRelAssociatesMaterial(props.ifcRelAssociatesMaterial) : undefined;
        this.ifcRelConnects = (props.ifcRelConnects) ? new IfcRelConnects(props.ifcRelConnects) : undefined;
        this.ifcRelConnectsElements = (props.ifcRelConnectsElements) ? new IfcRelConnectsElements(props.ifcRelConnectsElements) : undefined;
        this.ifcRelConnectsPathElements = (props.ifcRelConnectsPathElements) ? new IfcRelConnectsPathElements(props.ifcRelConnectsPathElements) : undefined;
        this.ifcRelConnectsPortToElement = (props.ifcRelConnectsPortToElement) ? new IfcRelConnectsPortToElement(props.ifcRelConnectsPortToElement) : undefined;
        this.ifcRelConnectsPorts = (props.ifcRelConnectsPorts) ? new IfcRelConnectsPorts(props.ifcRelConnectsPorts) : undefined;
        this.ifcRelConnectsStructuralActivity = (props.ifcRelConnectsStructuralActivity) ? new IfcRelConnectsStructuralActivity(props.ifcRelConnectsStructuralActivity) : undefined;
        this.ifcRelConnectsStructuralMember = (props.ifcRelConnectsStructuralMember) ? new IfcRelConnectsStructuralMember(props.ifcRelConnectsStructuralMember) : undefined;
        this.ifcRelConnectsWithEccentricity = (props.ifcRelConnectsWithEccentricity) ? new IfcRelConnectsWithEccentricity(props.ifcRelConnectsWithEccentricity) : undefined;
        this.ifcRelConnectsWithRealizingElements = (props.ifcRelConnectsWithRealizingElements) ? new IfcRelConnectsWithRealizingElements(props.ifcRelConnectsWithRealizingElements) : undefined;
        this.ifcRelContainedInSpatialStructure = (props.ifcRelContainedInSpatialStructure) ? new IfcRelContainedInSpatialStructure(props.ifcRelContainedInSpatialStructure) : undefined;
        this.ifcRelCoversBldgElements = (props.ifcRelCoversBldgElements) ? new IfcRelCoversBldgElements(props.ifcRelCoversBldgElements) : undefined;
        this.ifcRelCoversSpaces = (props.ifcRelCoversSpaces) ? new IfcRelCoversSpaces(props.ifcRelCoversSpaces) : undefined;
        this.ifcRelDeclares = (props.ifcRelDeclares) ? new IfcRelDeclares(props.ifcRelDeclares) : undefined;
        this.ifcRelDecomposes = (props.ifcRelDecomposes) ? new IfcRelDecomposes(props.ifcRelDecomposes) : undefined;
        this.ifcRelDefines = (props.ifcRelDefines) ? new IfcRelDefines(props.ifcRelDefines) : undefined;
        this.ifcRelDefinesByObject = props.ifcRelDefinesByObject;
        this.ifcRelDefinesByProperties = (props.ifcRelDefinesByProperties) ? new IfcRelDefinesByProperties(props.ifcRelDefinesByProperties) : undefined;
        this.ifcRelDefinesByTemplate = (props.ifcRelDefinesByTemplate) ? new IfcRelDefinesByTemplate(props.ifcRelDefinesByTemplate) : undefined;
        this.ifcRelDefinesByType = (props.ifcRelDefinesByType) ? new IfcRelDefinesByType(props.ifcRelDefinesByType) : undefined;
        this.ifcRelFillsElement = (props.ifcRelFillsElement) ? new IfcRelFillsElement(props.ifcRelFillsElement) : undefined;
        this.ifcRelFlowControlElements = (props.ifcRelFlowControlElements) ? new IfcRelFlowControlElements(props.ifcRelFlowControlElements) : undefined;
        this.ifcRelInterferesElements = (props.ifcRelInterferesElements) ? new IfcRelInterferesElements(props.ifcRelInterferesElements) : undefined;
        this.ifcRelNests = (props.ifcRelNests) ? new IfcRelNests(props.ifcRelNests) : undefined;
        this.ifcRelProjectsElement = (props.ifcRelProjectsElement) ? new IfcRelProjectsElement(props.ifcRelProjectsElement) : undefined;
        this.ifcRelReferencedInSpatialStructure = (props.ifcRelReferencedInSpatialStructure) ? new IfcRelReferencedInSpatialStructure(props.ifcRelReferencedInSpatialStructure) : undefined;
        this.ifcRelSequence = (props.ifcRelSequence) ? new IfcRelSequence(props.ifcRelSequence) : undefined;
        this.ifcRelServicesBuildings = (props.ifcRelServicesBuildings) ? new IfcRelServicesBuildings(props.ifcRelServicesBuildings) : undefined;
        this.ifcRelSpaceBoundary = (props.ifcRelSpaceBoundary) ? new IfcRelSpaceBoundary(props.ifcRelSpaceBoundary) : undefined;
        this.ifcRelSpaceBoundary1stLevel = (props.ifcRelSpaceBoundary1stLevel) ? new IfcRelSpaceBoundary1stLevel(props.ifcRelSpaceBoundary1stLevel) : undefined;
        this.ifcRelSpaceBoundary2ndLevel = (props.ifcRelSpaceBoundary2ndLevel) ? new IfcRelSpaceBoundary2ndLevel(props.ifcRelSpaceBoundary2ndLevel) : undefined;
        this.ifcRelVoidsElement = (props.ifcRelVoidsElement) ? new IfcRelVoidsElement(props.ifcRelVoidsElement) : undefined;
        this.ifcRelationship = (props.ifcRelationship) ? new IfcRelationship(props.ifcRelationship) : undefined;
        this.ifcReparametrisedCompositeCurveSegment = (props.ifcReparametrisedCompositeCurveSegment) ? new IfcReparametrisedCompositeCurveSegment(props.ifcReparametrisedCompositeCurveSegment) : undefined;
        this.ifcRepresentation = (props.ifcRepresentation) ? new IfcRepresentation(props.ifcRepresentation) : undefined;
        this.ifcRepresentationContext = (props.ifcRepresentationContext) ? new IfcRepresentationContext(props.ifcRepresentationContext) : undefined;
        this.ifcRepresentationItem = (props.ifcRepresentationItem) ? new IfcRepresentationItem(props.ifcRepresentationItem) : undefined;
        this.ifcRepresentationMap = (props.ifcRepresentationMap) ? new IfcRepresentationMap(props.ifcRepresentationMap) : undefined;
        this.ifcResource = (props.ifcResource) ? new IfcResource(props.ifcResource) : undefined;
        this.ifcResourceApprovalRelationship = (props.ifcResourceApprovalRelationship) ? new IfcResourceApprovalRelationship(props.ifcResourceApprovalRelationship) : undefined;
        this.ifcResourceConstraintRelationship = (props.ifcResourceConstraintRelationship) ? new IfcResourceConstraintRelationship(props.ifcResourceConstraintRelationship) : undefined;
        this.ifcResourceLevelRelationship = (props.ifcResourceLevelRelationship) ? new IfcResourceLevelRelationship(props.ifcResourceLevelRelationship) : undefined;
        this.ifcResourceTime = (props.ifcResourceTime) ? new IfcResourceTime(props.ifcResourceTime) : undefined;
        this.ifcRevolvedAreaSolid = (props.ifcRevolvedAreaSolid) ? new IfcRevolvedAreaSolid(props.ifcRevolvedAreaSolid) : undefined;
        this.ifcRevolvedAreaSolidTapered = (props.ifcRevolvedAreaSolidTapered) ? new IfcRevolvedAreaSolidTapered(props.ifcRevolvedAreaSolidTapered) : undefined;
        this.ifcRightCircularCone = (props.ifcRightCircularCone) ? new IfcRightCircularCone(props.ifcRightCircularCone) : undefined;
        this.ifcRightCircularCylinder = (props.ifcRightCircularCylinder) ? new IfcRightCircularCylinder(props.ifcRightCircularCylinder) : undefined;
        this.ifcRoof = (props.ifcRoof) ? new IfcRoof(props.ifcRoof) : undefined;
        this.ifcRoofType = (props.ifcRoofType) ? new IfcRoofType(props.ifcRoofType) : undefined;
        this.ifcRoot = (props.ifcRoot) ? new IfcRoot(props.ifcRoot) : undefined;
        this.ifcRoundedRectangleProfileDef = (props.ifcRoundedRectangleProfileDef) ? new IfcRoundedRectangleProfileDef(props.ifcRoundedRectangleProfileDef) : undefined;
        this.ifcSIUnit = (props.ifcSIUnit) ? new IfcSIUnit(props.ifcSIUnit) : undefined;
        this.ifcSanitaryTerminal = (props.ifcSanitaryTerminal) ? new IfcSanitaryTerminal(props.ifcSanitaryTerminal) : undefined;
        this.ifcSanitaryTerminalType = (props.ifcSanitaryTerminalType) ? new IfcSanitaryTerminalType(props.ifcSanitaryTerminalType) : undefined;
        this.ifcSchedulingTime = (props.ifcSchedulingTime) ? new IfcSchedulingTime(props.ifcSchedulingTime) : undefined;
        this.ifcSeamCurve = (props.ifcSeamCurve) ? new IfcSeamCurve(props.ifcSeamCurve) : undefined;
        this.ifcSectionProperties = (props.ifcSectionProperties) ? new IfcSectionProperties(props.ifcSectionProperties) : undefined;
        this.ifcSectionReinforcementProperties = (props.ifcSectionReinforcementProperties) ? new IfcSectionReinforcementProperties(props.ifcSectionReinforcementProperties) : undefined;
        this.ifcSectionedSpine = (props.ifcSectionedSpine) ? new IfcSectionedSpine(props.ifcSectionedSpine) : undefined;
        this.ifcSensor = (props.ifcSensor) ? new IfcSensor(props.ifcSensor) : undefined;
        this.ifcSensorType = (props.ifcSensorType) ? new IfcSensorType(props.ifcSensorType) : undefined;
        this.ifcShadingDevice = (props.ifcShadingDevice) ? new IfcShadingDevice(props.ifcShadingDevice) : undefined;
        this.ifcShadingDeviceType = (props.ifcShadingDeviceType) ? new IfcShadingDeviceType(props.ifcShadingDeviceType) : undefined;
        this.ifcShapeAspect = props.ifcShapeAspect;
        this.ifcShapeModel = (props.ifcShapeModel) ? new IfcShapeModel(props.ifcShapeModel) : undefined;
        this.ifcShapeRepresentation = (props.ifcShapeRepresentation) ? new IfcShapeRepresentation(props.ifcShapeRepresentation) : undefined;
        this.ifcShellBasedSurfaceModel = (props.ifcShellBasedSurfaceModel) ? new IfcShellBasedSurfaceModel(props.ifcShellBasedSurfaceModel) : undefined;
        this.ifcSimpleProperty = (props.ifcSimpleProperty) ? new IfcSimpleProperty(props.ifcSimpleProperty) : undefined;
        this.ifcSimplePropertyTemplate = (props.ifcSimplePropertyTemplate) ? new IfcSimplePropertyTemplate(props.ifcSimplePropertyTemplate) : undefined;
        this.ifcSite = (props.ifcSite) ? new IfcSite(props.ifcSite) : undefined;
        this.ifcSlab = (props.ifcSlab) ? new IfcSlab(props.ifcSlab) : undefined;
        this.ifcSlabElementedCase = (props.ifcSlabElementedCase) ? new IfcSlabElementedCase(props.ifcSlabElementedCase) : undefined;
        this.ifcSlabStandardCase = (props.ifcSlabStandardCase) ? new IfcSlabStandardCase(props.ifcSlabStandardCase) : undefined;
        this.ifcSlabType = (props.ifcSlabType) ? new IfcSlabType(props.ifcSlabType) : undefined;
        this.ifcSlippageConnectionCondition = (props.ifcSlippageConnectionCondition) ? new IfcSlippageConnectionCondition(props.ifcSlippageConnectionCondition) : undefined;
        this.ifcSolarDevice = (props.ifcSolarDevice) ? new IfcSolarDevice(props.ifcSolarDevice) : undefined;
        this.ifcSolarDeviceType = (props.ifcSolarDeviceType) ? new IfcSolarDeviceType(props.ifcSolarDeviceType) : undefined;
        this.ifcSolidModel = (props.ifcSolidModel) ? new IfcSolidModel(props.ifcSolidModel) : undefined;
        this.ifcSpace = (props.ifcSpace) ? new IfcSpace(props.ifcSpace) : undefined;
        this.ifcSpaceHeater = (props.ifcSpaceHeater) ? new IfcSpaceHeater(props.ifcSpaceHeater) : undefined;
        this.ifcSpaceHeaterType = (props.ifcSpaceHeaterType) ? new IfcSpaceHeaterType(props.ifcSpaceHeaterType) : undefined;
        this.ifcSpaceType = (props.ifcSpaceType) ? new IfcSpaceType(props.ifcSpaceType) : undefined;
        this.ifcSpatialElement = (props.ifcSpatialElement) ? new IfcSpatialElement(props.ifcSpatialElement) : undefined;
        this.ifcSpatialElementType = (props.ifcSpatialElementType) ? new IfcSpatialElementType(props.ifcSpatialElementType) : undefined;
        this.ifcSpatialStructureElement = (props.ifcSpatialStructureElement) ? new IfcSpatialStructureElement(props.ifcSpatialStructureElement) : undefined;
        this.ifcSpatialStructureElementType = (props.ifcSpatialStructureElementType) ? new IfcSpatialStructureElementType(props.ifcSpatialStructureElementType) : undefined;
        this.ifcSpatialZone = (props.ifcSpatialZone) ? new IfcSpatialZone(props.ifcSpatialZone) : undefined;
        this.ifcSpatialZoneType = (props.ifcSpatialZoneType) ? new IfcSpatialZoneType(props.ifcSpatialZoneType) : undefined;
        this.ifcSphere = (props.ifcSphere) ? new IfcSphere(props.ifcSphere) : undefined;
        this.ifcSphericalSurface = (props.ifcSphericalSurface) ? new IfcSphericalSurface(props.ifcSphericalSurface) : undefined;
        this.ifcStackTerminal = (props.ifcStackTerminal) ? new IfcStackTerminal(props.ifcStackTerminal) : undefined;
        this.ifcStackTerminalType = (props.ifcStackTerminalType) ? new IfcStackTerminalType(props.ifcStackTerminalType) : undefined;
        this.ifcStair = (props.ifcStair) ? new IfcStair(props.ifcStair) : undefined;
        this.ifcStairFlight = (props.ifcStairFlight) ? new IfcStairFlight(props.ifcStairFlight) : undefined;
        this.ifcStairFlightType = (props.ifcStairFlightType) ? new IfcStairFlightType(props.ifcStairFlightType) : undefined;
        this.ifcStairType = (props.ifcStairType) ? new IfcStairType(props.ifcStairType) : undefined;
        this.ifcStructuralAction = (props.ifcStructuralAction) ? new IfcStructuralAction(props.ifcStructuralAction) : undefined;
        this.ifcStructuralActivity = (props.ifcStructuralActivity) ? new IfcStructuralActivity(props.ifcStructuralActivity) : undefined;
        this.ifcStructuralAnalysisModel = (props.ifcStructuralAnalysisModel) ? new IfcStructuralAnalysisModel(props.ifcStructuralAnalysisModel) : undefined;
        this.ifcStructuralConnection = (props.ifcStructuralConnection) ? new IfcStructuralConnection(props.ifcStructuralConnection) : undefined;
        this.ifcStructuralConnectionCondition = (props.ifcStructuralConnectionCondition) ? new IfcStructuralConnectionCondition(props.ifcStructuralConnectionCondition) : undefined;
        this.ifcStructuralCurveAction = (props.ifcStructuralCurveAction) ? new IfcStructuralCurveAction(props.ifcStructuralCurveAction) : undefined;
        this.ifcStructuralCurveConnection = (props.ifcStructuralCurveConnection) ? new IfcStructuralCurveConnection(props.ifcStructuralCurveConnection) : undefined;
        this.ifcStructuralCurveMember = (props.ifcStructuralCurveMember) ? new IfcStructuralCurveMember(props.ifcStructuralCurveMember) : undefined;
        this.ifcStructuralCurveMemberVarying = (props.ifcStructuralCurveMemberVarying) ? new IfcStructuralCurveMemberVarying(props.ifcStructuralCurveMemberVarying) : undefined;
        this.ifcStructuralCurveReaction = (props.ifcStructuralCurveReaction) ? new IfcStructuralCurveReaction(props.ifcStructuralCurveReaction) : undefined;
        this.ifcStructuralItem = (props.ifcStructuralItem) ? new IfcStructuralItem(props.ifcStructuralItem) : undefined;
        this.ifcStructuralLinearAction = (props.ifcStructuralLinearAction) ? new IfcStructuralLinearAction(props.ifcStructuralLinearAction) : undefined;
        this.ifcStructuralLoad = (props.ifcStructuralLoad) ? new IfcStructuralLoad(props.ifcStructuralLoad) : undefined;
        this.ifcStructuralLoadCase = (props.ifcStructuralLoadCase) ? new IfcStructuralLoadCase(props.ifcStructuralLoadCase) : undefined;
        this.ifcStructuralLoadConfiguration = (props.ifcStructuralLoadConfiguration) ? new IfcStructuralLoadConfiguration(props.ifcStructuralLoadConfiguration) : undefined;
        this.ifcStructuralLoadGroup = (props.ifcStructuralLoadGroup) ? new IfcStructuralLoadGroup(props.ifcStructuralLoadGroup) : undefined;
        this.ifcStructuralLoadLinearForce = (props.ifcStructuralLoadLinearForce) ? new IfcStructuralLoadLinearForce(props.ifcStructuralLoadLinearForce) : undefined;
        this.ifcStructuralLoadOrResult = (props.ifcStructuralLoadOrResult) ? new IfcStructuralLoadOrResult(props.ifcStructuralLoadOrResult) : undefined;
        this.ifcStructuralLoadPlanarForce = (props.ifcStructuralLoadPlanarForce) ? new IfcStructuralLoadPlanarForce(props.ifcStructuralLoadPlanarForce) : undefined;
        this.ifcStructuralLoadSingleDisplacement = (props.ifcStructuralLoadSingleDisplacement) ? new IfcStructuralLoadSingleDisplacement(props.ifcStructuralLoadSingleDisplacement) : undefined;
        this.ifcStructuralLoadSingleDisplacementDistortion = (props.ifcStructuralLoadSingleDisplacementDistortion) ? new IfcStructuralLoadSingleDisplacementDistortion(props.ifcStructuralLoadSingleDisplacementDistortion) : undefined;
        this.ifcStructuralLoadSingleForce = (props.ifcStructuralLoadSingleForce) ? new IfcStructuralLoadSingleForce(props.ifcStructuralLoadSingleForce) : undefined;
        this.ifcStructuralLoadSingleForceWarping = (props.ifcStructuralLoadSingleForceWarping) ? new IfcStructuralLoadSingleForceWarping(props.ifcStructuralLoadSingleForceWarping) : undefined;
        this.ifcStructuralLoadStatic = (props.ifcStructuralLoadStatic) ? new IfcStructuralLoadStatic(props.ifcStructuralLoadStatic) : undefined;
        this.ifcStructuralLoadTemperature = (props.ifcStructuralLoadTemperature) ? new IfcStructuralLoadTemperature(props.ifcStructuralLoadTemperature) : undefined;
        this.ifcStructuralMember = (props.ifcStructuralMember) ? new IfcStructuralMember(props.ifcStructuralMember) : undefined;
        this.ifcStructuralPlanarAction = (props.ifcStructuralPlanarAction) ? new IfcStructuralPlanarAction(props.ifcStructuralPlanarAction) : undefined;
        this.ifcStructuralPointAction = (props.ifcStructuralPointAction) ? new IfcStructuralPointAction(props.ifcStructuralPointAction) : undefined;
        this.ifcStructuralPointConnection = (props.ifcStructuralPointConnection) ? new IfcStructuralPointConnection(props.ifcStructuralPointConnection) : undefined;
        this.ifcStructuralPointReaction = (props.ifcStructuralPointReaction) ? new IfcStructuralPointReaction(props.ifcStructuralPointReaction) : undefined;
        this.ifcStructuralReaction = (props.ifcStructuralReaction) ? new IfcStructuralReaction(props.ifcStructuralReaction) : undefined;
        this.ifcStructuralResultGroup = (props.ifcStructuralResultGroup) ? new IfcStructuralResultGroup(props.ifcStructuralResultGroup) : undefined;
        this.ifcStructuralSurfaceAction = (props.ifcStructuralSurfaceAction) ? new IfcStructuralSurfaceAction(props.ifcStructuralSurfaceAction) : undefined;
        this.ifcStructuralSurfaceConnection = (props.ifcStructuralSurfaceConnection) ? new IfcStructuralSurfaceConnection(props.ifcStructuralSurfaceConnection) : undefined;
        this.ifcStructuralSurfaceMember = (props.ifcStructuralSurfaceMember) ? new IfcStructuralSurfaceMember(props.ifcStructuralSurfaceMember) : undefined;
        this.ifcStructuralSurfaceMemberVarying = (props.ifcStructuralSurfaceMemberVarying) ? new IfcStructuralSurfaceMemberVarying(props.ifcStructuralSurfaceMemberVarying) : undefined;
        this.ifcStructuralSurfaceReaction = (props.ifcStructuralSurfaceReaction) ? new IfcStructuralSurfaceReaction(props.ifcStructuralSurfaceReaction) : undefined;
        this.ifcStyleModel = (props.ifcStyleModel) ? new IfcStyleModel(props.ifcStyleModel) : undefined;
        this.ifcStyledItem = (props.ifcStyledItem) ? new IfcStyledItem(props.ifcStyledItem) : undefined;
        this.ifcStyledRepresentation = (props.ifcStyledRepresentation) ? new IfcStyledRepresentation(props.ifcStyledRepresentation) : undefined;
        this.ifcSubContractResource = (props.ifcSubContractResource) ? new IfcSubContractResource(props.ifcSubContractResource) : undefined;
        this.ifcSubContractResourceType = (props.ifcSubContractResourceType) ? new IfcSubContractResourceType(props.ifcSubContractResourceType) : undefined;
        this.ifcSubedge = (props.ifcSubedge) ? new IfcSubedge(props.ifcSubedge) : undefined;
        this.ifcSurface = (props.ifcSurface) ? new IfcSurface(props.ifcSurface) : undefined;
        this.ifcSurfaceCurve = (props.ifcSurfaceCurve) ? new IfcSurfaceCurve(props.ifcSurfaceCurve) : undefined;
        this.ifcSurfaceCurveSweptAreaSolid = (props.ifcSurfaceCurveSweptAreaSolid) ? new IfcSurfaceCurveSweptAreaSolid(props.ifcSurfaceCurveSweptAreaSolid) : undefined;
        this.ifcSurfaceFeature = (props.ifcSurfaceFeature) ? new IfcSurfaceFeature(props.ifcSurfaceFeature) : undefined;
        this.ifcSurfaceOfLinearExtrusion = (props.ifcSurfaceOfLinearExtrusion) ? new IfcSurfaceOfLinearExtrusion(props.ifcSurfaceOfLinearExtrusion) : undefined;
        this.ifcSurfaceOfRevolution = (props.ifcSurfaceOfRevolution) ? new IfcSurfaceOfRevolution(props.ifcSurfaceOfRevolution) : undefined;
        this.ifcSurfaceReinforcementArea = (props.ifcSurfaceReinforcementArea) ? new IfcSurfaceReinforcementArea(props.ifcSurfaceReinforcementArea) : undefined;
        this.ifcSurfaceStyle = (props.ifcSurfaceStyle) ? new IfcSurfaceStyle(props.ifcSurfaceStyle) : undefined;
        this.ifcSurfaceStyleLighting = (props.ifcSurfaceStyleLighting) ? new IfcSurfaceStyleLighting(props.ifcSurfaceStyleLighting) : undefined;
        this.ifcSurfaceStyleRefraction = (props.ifcSurfaceStyleRefraction) ? new IfcSurfaceStyleRefraction(props.ifcSurfaceStyleRefraction) : undefined;
        this.ifcSurfaceStyleRendering = (props.ifcSurfaceStyleRendering) ? new IfcSurfaceStyleRendering(props.ifcSurfaceStyleRendering) : undefined;
        this.ifcSurfaceStyleShading = (props.ifcSurfaceStyleShading) ? new IfcSurfaceStyleShading(props.ifcSurfaceStyleShading) : undefined;
        this.ifcSurfaceStyleWithTextures = (props.ifcSurfaceStyleWithTextures) ? new IfcSurfaceStyleWithTextures(props.ifcSurfaceStyleWithTextures) : undefined;
        this.ifcSurfaceTexture = (props.ifcSurfaceTexture) ? new IfcSurfaceTexture(props.ifcSurfaceTexture) : undefined;
        this.ifcSweptAreaSolid = (props.ifcSweptAreaSolid) ? new IfcSweptAreaSolid(props.ifcSweptAreaSolid) : undefined;
        this.ifcSweptDiskSolid = (props.ifcSweptDiskSolid) ? new IfcSweptDiskSolid(props.ifcSweptDiskSolid) : undefined;
        this.ifcSweptDiskSolidPolygonal = (props.ifcSweptDiskSolidPolygonal) ? new IfcSweptDiskSolidPolygonal(props.ifcSweptDiskSolidPolygonal) : undefined;
        this.ifcSweptSurface = (props.ifcSweptSurface) ? new IfcSweptSurface(props.ifcSweptSurface) : undefined;
        this.ifcSwitchingDevice = (props.ifcSwitchingDevice) ? new IfcSwitchingDevice(props.ifcSwitchingDevice) : undefined;
        this.ifcSwitchingDeviceType = (props.ifcSwitchingDeviceType) ? new IfcSwitchingDeviceType(props.ifcSwitchingDeviceType) : undefined;
        this.ifcSystem = (props.ifcSystem) ? new IfcSystem(props.ifcSystem) : undefined;
        this.ifcSystemFurnitureElement = (props.ifcSystemFurnitureElement) ? new IfcSystemFurnitureElement(props.ifcSystemFurnitureElement) : undefined;
        this.ifcSystemFurnitureElementType = (props.ifcSystemFurnitureElementType) ? new IfcSystemFurnitureElementType(props.ifcSystemFurnitureElementType) : undefined;
        this.ifcTShapeProfileDef = (props.ifcTShapeProfileDef) ? new IfcTShapeProfileDef(props.ifcTShapeProfileDef) : undefined;
        this.ifcTable = (props.ifcTable) ? new IfcTable(props.ifcTable) : undefined;
        this.ifcTableColumn = (props.ifcTableColumn) ? new IfcTableColumn(props.ifcTableColumn) : undefined;
        this.ifcTableRow = (props.ifcTableRow) ? new IfcTableRow(props.ifcTableRow) : undefined;
        this.ifcTank = (props.ifcTank) ? new IfcTank(props.ifcTank) : undefined;
        this.ifcTankType = (props.ifcTankType) ? new IfcTankType(props.ifcTankType) : undefined;
        this.ifcTask = (props.ifcTask) ? new IfcTask(props.ifcTask) : undefined;
        this.ifcTaskTime = (props.ifcTaskTime) ? new IfcTaskTime(props.ifcTaskTime) : undefined;
        this.ifcTaskTimeRecurring = (props.ifcTaskTimeRecurring) ? new IfcTaskTimeRecurring(props.ifcTaskTimeRecurring) : undefined;
        this.ifcTaskType = (props.ifcTaskType) ? new IfcTaskType(props.ifcTaskType) : undefined;
        this.ifcTelecomAddress = (props.ifcTelecomAddress) ? new IfcTelecomAddress(props.ifcTelecomAddress) : undefined;
        this.ifcTendon = (props.ifcTendon) ? new IfcTendon(props.ifcTendon) : undefined;
        this.ifcTendonAnchor = (props.ifcTendonAnchor) ? new IfcTendonAnchor(props.ifcTendonAnchor) : undefined;
        this.ifcTendonAnchorType = (props.ifcTendonAnchorType) ? new IfcTendonAnchorType(props.ifcTendonAnchorType) : undefined;
        this.ifcTendonType = (props.ifcTendonType) ? new IfcTendonType(props.ifcTendonType) : undefined;
        this.ifcTessellatedFaceSet = (props.ifcTessellatedFaceSet) ? new IfcTessellatedFaceSet(props.ifcTessellatedFaceSet) : undefined;
        this.ifcTessellatedItem = (props.ifcTessellatedItem) ? new IfcTessellatedItem(props.ifcTessellatedItem) : undefined;
        this.ifcTextLiteral = (props.ifcTextLiteral) ? new IfcTextLiteral(props.ifcTextLiteral) : undefined;
        this.ifcTextLiteralWithExtent = (props.ifcTextLiteralWithExtent) ? new IfcTextLiteralWithExtent(props.ifcTextLiteralWithExtent) : undefined;
        this.ifcTextStyle = (props.ifcTextStyle) ? new IfcTextStyle(props.ifcTextStyle) : undefined;
        this.ifcTextStyleFontModel = (props.ifcTextStyleFontModel) ? new IfcTextStyleFontModel(props.ifcTextStyleFontModel) : undefined;
        this.ifcTextStyleForDefinedFont = (props.ifcTextStyleForDefinedFont) ? new IfcTextStyleForDefinedFont(props.ifcTextStyleForDefinedFont) : undefined;
        this.ifcTextStyleTextModel = (props.ifcTextStyleTextModel) ? new IfcTextStyleTextModel(props.ifcTextStyleTextModel) : undefined;
        this.ifcTextureCoordinate = (props.ifcTextureCoordinate) ? new IfcTextureCoordinate(props.ifcTextureCoordinate) : undefined;
        this.ifcTextureCoordinateGenerator = (props.ifcTextureCoordinateGenerator) ? new IfcTextureCoordinateGenerator(props.ifcTextureCoordinateGenerator) : undefined;
        this.ifcTextureMap = (props.ifcTextureMap) ? new IfcTextureMap(props.ifcTextureMap) : undefined;
        this.ifcTextureVertex = (props.ifcTextureVertex) ? new IfcTextureVertex(props.ifcTextureVertex) : undefined;
        this.ifcTextureVertexList = (props.ifcTextureVertexList) ? new IfcTextureVertexList(props.ifcTextureVertexList) : undefined;
        this.ifcTimePeriod = (props.ifcTimePeriod) ? new IfcTimePeriod(props.ifcTimePeriod) : undefined;
        this.ifcTimeSeries = (props.ifcTimeSeries) ? new IfcTimeSeries(props.ifcTimeSeries) : undefined;
        this.ifcTimeSeriesValue = (props.ifcTimeSeriesValue) ? new IfcTimeSeriesValue(props.ifcTimeSeriesValue) : undefined;
        this.ifcTopologicalRepresentationItem = (props.ifcTopologicalRepresentationItem) ? new IfcTopologicalRepresentationItem(props.ifcTopologicalRepresentationItem) : undefined;
        this.ifcTopologyRepresentation = (props.ifcTopologyRepresentation) ? new IfcTopologyRepresentation(props.ifcTopologyRepresentation) : undefined;
        this.ifcToroidalSurface = (props.ifcToroidalSurface) ? new IfcToroidalSurface(props.ifcToroidalSurface) : undefined;
        this.ifcTransformer = (props.ifcTransformer) ? new IfcTransformer(props.ifcTransformer) : undefined;
        this.ifcTransformerType = (props.ifcTransformerType) ? new IfcTransformerType(props.ifcTransformerType) : undefined;
        this.ifcTransportElement = (props.ifcTransportElement) ? new IfcTransportElement(props.ifcTransportElement) : undefined;
        this.ifcTransportElementType = (props.ifcTransportElementType) ? new IfcTransportElementType(props.ifcTransportElementType) : undefined;
        this.ifcTrapeziumProfileDef = (props.ifcTrapeziumProfileDef) ? new IfcTrapeziumProfileDef(props.ifcTrapeziumProfileDef) : undefined;
        this.ifcTriangulatedFaceSet = (props.ifcTriangulatedFaceSet) ? new IfcTriangulatedFaceSet(props.ifcTriangulatedFaceSet) : undefined;
        this.ifcTrimmedCurve = (props.ifcTrimmedCurve) ? new IfcTrimmedCurve(props.ifcTrimmedCurve) : undefined;
        this.ifcTubeBundle = (props.ifcTubeBundle) ? new IfcTubeBundle(props.ifcTubeBundle) : undefined;
        this.ifcTubeBundleType = (props.ifcTubeBundleType) ? new IfcTubeBundleType(props.ifcTubeBundleType) : undefined;
        this.ifcTypeObject = (props.ifcTypeObject) ? new IfcTypeObject(props.ifcTypeObject) : undefined;
        this.ifcTypeProcess = (props.ifcTypeProcess) ? new IfcTypeProcess(props.ifcTypeProcess) : undefined;
        this.ifcTypeProduct = (props.ifcTypeProduct) ? new IfcTypeProduct(props.ifcTypeProduct) : undefined;
        this.ifcTypeResource = (props.ifcTypeResource) ? new IfcTypeResource(props.ifcTypeResource) : undefined;
        this.ifcUShapeProfileDef = (props.ifcUShapeProfileDef) ? new IfcUShapeProfileDef(props.ifcUShapeProfileDef) : undefined;
        this.ifcUnitAssignment = (props.ifcUnitAssignment) ? new IfcUnitAssignment(props.ifcUnitAssignment) : undefined;
        this.ifcUnitaryControlElement = (props.ifcUnitaryControlElement) ? new IfcUnitaryControlElement(props.ifcUnitaryControlElement) : undefined;
        this.ifcUnitaryControlElementType = (props.ifcUnitaryControlElementType) ? new IfcUnitaryControlElementType(props.ifcUnitaryControlElementType) : undefined;
        this.ifcUnitaryEquipment = (props.ifcUnitaryEquipment) ? new IfcUnitaryEquipment(props.ifcUnitaryEquipment) : undefined;
        this.ifcUnitaryEquipmentType = (props.ifcUnitaryEquipmentType) ? new IfcUnitaryEquipmentType(props.ifcUnitaryEquipmentType) : undefined;
        this.ifcValve = (props.ifcValve) ? new IfcValve(props.ifcValve) : undefined;
        this.ifcValveType = (props.ifcValveType) ? new IfcValveType(props.ifcValveType) : undefined;
        this.ifcVector = (props.ifcVector) ? new IfcVector(props.ifcVector) : undefined;
        this.ifcVertex = (props.ifcVertex) ? new IfcVertex(props.ifcVertex) : undefined;
        this.ifcVertexLoop = (props.ifcVertexLoop) ? new IfcVertexLoop(props.ifcVertexLoop) : undefined;
        this.ifcVertexPoint = (props.ifcVertexPoint) ? new IfcVertexPoint(props.ifcVertexPoint) : undefined;
        this.ifcVibrationIsolator = (props.ifcVibrationIsolator) ? new IfcVibrationIsolator(props.ifcVibrationIsolator) : undefined;
        this.ifcVibrationIsolatorType = (props.ifcVibrationIsolatorType) ? new IfcVibrationIsolatorType(props.ifcVibrationIsolatorType) : undefined;
        this.ifcVirtualElement = (props.ifcVirtualElement) ? new IfcVirtualElement(props.ifcVirtualElement) : undefined;
        this.ifcVirtualGridIntersection = (props.ifcVirtualGridIntersection) ? new IfcVirtualGridIntersection(props.ifcVirtualGridIntersection) : undefined;
        this.ifcVoidingFeature = (props.ifcVoidingFeature) ? new IfcVoidingFeature(props.ifcVoidingFeature) : undefined;
        this.ifcWall = (props.ifcWall) ? new IfcWall(props.ifcWall) : undefined;
        this.ifcWallElementedCase = (props.ifcWallElementedCase) ? new IfcWallElementedCase(props.ifcWallElementedCase) : undefined;
        this.ifcWallStandardCase = (props.ifcWallStandardCase) ? new IfcWallStandardCase(props.ifcWallStandardCase) : undefined;
        this.ifcWallType = (props.ifcWallType) ? new IfcWallType(props.ifcWallType) : undefined;
        this.ifcWasteTerminal = (props.ifcWasteTerminal) ? new IfcWasteTerminal(props.ifcWasteTerminal) : undefined;
        this.ifcWasteTerminalType = (props.ifcWasteTerminalType) ? new IfcWasteTerminalType(props.ifcWasteTerminalType) : undefined;
        this.ifcWindow = (props.ifcWindow) ? new IfcWindow(props.ifcWindow) : undefined;
        this.ifcWindowLiningProperties = (props.ifcWindowLiningProperties) ? new IfcWindowLiningProperties(props.ifcWindowLiningProperties) : undefined;
        this.ifcWindowPanelProperties = (props.ifcWindowPanelProperties) ? new IfcWindowPanelProperties(props.ifcWindowPanelProperties) : undefined;
        this.ifcWindowStandardCase = (props.ifcWindowStandardCase) ? new IfcWindowStandardCase(props.ifcWindowStandardCase) : undefined;
        this.ifcWindowStyle = (props.ifcWindowStyle) ? new IfcWindowStyle(props.ifcWindowStyle) : undefined;
        this.ifcWindowType = (props.ifcWindowType) ? new IfcWindowType(props.ifcWindowType) : undefined;
        this.ifcWorkCalendar = (props.ifcWorkCalendar) ? new IfcWorkCalendar(props.ifcWorkCalendar) : undefined;
        this.ifcWorkControl = (props.ifcWorkControl) ? new IfcWorkControl(props.ifcWorkControl) : undefined;
        this.ifcWorkPlan = (props.ifcWorkPlan) ? new IfcWorkPlan(props.ifcWorkPlan) : undefined;
        this.ifcWorkSchedule = (props.ifcWorkSchedule) ? new IfcWorkSchedule(props.ifcWorkSchedule) : undefined;
        this.ifcWorkTime = (props.ifcWorkTime) ? new IfcWorkTime(props.ifcWorkTime) : undefined;
        this.ifcZShapeProfileDef = (props.ifcZShapeProfileDef) ? new IfcZShapeProfileDef(props.ifcZShapeProfileDef) : undefined;
        this.ifcZone = (props.ifcZone) ? new IfcZone(props.ifcZone) : undefined;
        this.entity = (props.entity) ? new Entity(props.entity) : undefined;
        this.ifcAbsorbedDoseMeasure - wrapper;
        (props.ifcAbsorbedDoseMeasure - wrapper) ? new IfcAbsorbedDoseMeasure - wrapper(props.ifcAbsorbedDoseMeasure - wrapper) : undefined;
        this.ifcAccelerationMeasure - wrapper;
        (props.ifcAccelerationMeasure - wrapper) ? new IfcAccelerationMeasure - wrapper(props.ifcAccelerationMeasure - wrapper) : undefined;
        this.ifcAmountOfSubstanceMeasure - wrapper;
        (props.ifcAmountOfSubstanceMeasure - wrapper) ? new IfcAmountOfSubstanceMeasure - wrapper(props.ifcAmountOfSubstanceMeasure - wrapper) : undefined;
        this.ifcAngularVelocityMeasure - wrapper;
        (props.ifcAngularVelocityMeasure - wrapper) ? new IfcAngularVelocityMeasure - wrapper(props.ifcAngularVelocityMeasure - wrapper) : undefined;
        this.ifcArcIndex - wrapper;
        (props.ifcArcIndex - wrapper) ? new IfcArcIndex - wrapper(props.ifcArcIndex - wrapper) : undefined;
        this.ifcAreaDensityMeasure - wrapper;
        (props.ifcAreaDensityMeasure - wrapper) ? new IfcAreaDensityMeasure - wrapper(props.ifcAreaDensityMeasure - wrapper) : undefined;
        this.ifcAreaMeasure - wrapper;
        (props.ifcAreaMeasure - wrapper) ? new IfcAreaMeasure - wrapper(props.ifcAreaMeasure - wrapper) : undefined;
        this.ifcBinary - wrapper;
        (props.ifcBinary - wrapper) ? new IfcBinary - wrapper(props.ifcBinary - wrapper) : undefined;
        this.ifcBoolean - wrapper;
        (props.ifcBoolean - wrapper) ? new IfcBoolean - wrapper(props.ifcBoolean - wrapper) : undefined;
        this.ifcComplexNumber - wrapper;
        (props.ifcComplexNumber - wrapper) ? new IfcComplexNumber - wrapper(props.ifcComplexNumber - wrapper) : undefined;
        this.ifcCompoundPlaneAngleMeasure - wrapper;
        (props.ifcCompoundPlaneAngleMeasure - wrapper) ? new IfcCompoundPlaneAngleMeasure - wrapper(props.ifcCompoundPlaneAngleMeasure - wrapper) : undefined;
        this.ifcContextDependentMeasure - wrapper;
        (props.ifcContextDependentMeasure - wrapper) ? new IfcContextDependentMeasure - wrapper(props.ifcContextDependentMeasure - wrapper) : undefined;
        this.ifcCountMeasure - wrapper;
        (props.ifcCountMeasure - wrapper) ? new IfcCountMeasure - wrapper(props.ifcCountMeasure - wrapper) : undefined;
        this.ifcCurvatureMeasure - wrapper;
        (props.ifcCurvatureMeasure - wrapper) ? new IfcCurvatureMeasure - wrapper(props.ifcCurvatureMeasure - wrapper) : undefined;
        this.ifcDate - wrapper;
        (props.ifcDate - wrapper) ? new IfcDate - wrapper(props.ifcDate - wrapper) : undefined;
        this.ifcDateTime - wrapper;
        (props.ifcDateTime - wrapper) ? new IfcDateTime - wrapper(props.ifcDateTime - wrapper) : undefined;
        this.ifcDescriptiveMeasure - wrapper;
        (props.ifcDescriptiveMeasure - wrapper) ? new IfcDescriptiveMeasure - wrapper(props.ifcDescriptiveMeasure - wrapper) : undefined;
        this.ifcDoseEquivalentMeasure - wrapper;
        (props.ifcDoseEquivalentMeasure - wrapper) ? new IfcDoseEquivalentMeasure - wrapper(props.ifcDoseEquivalentMeasure - wrapper) : undefined;
        this.ifcDuration - wrapper;
        (props.ifcDuration - wrapper) ? new IfcDuration - wrapper(props.ifcDuration - wrapper) : undefined;
        this.ifcDynamicViscosityMeasure - wrapper;
        (props.ifcDynamicViscosityMeasure - wrapper) ? new IfcDynamicViscosityMeasure - wrapper(props.ifcDynamicViscosityMeasure - wrapper) : undefined;
        this.ifcElectricCapacitanceMeasure - wrapper;
        (props.ifcElectricCapacitanceMeasure - wrapper) ? new IfcElectricCapacitanceMeasure - wrapper(props.ifcElectricCapacitanceMeasure - wrapper) : undefined;
        this.ifcElectricChargeMeasure - wrapper;
        (props.ifcElectricChargeMeasure - wrapper) ? new IfcElectricChargeMeasure - wrapper(props.ifcElectricChargeMeasure - wrapper) : undefined;
        this.ifcElectricConductanceMeasure - wrapper;
        (props.ifcElectricConductanceMeasure - wrapper) ? new IfcElectricConductanceMeasure - wrapper(props.ifcElectricConductanceMeasure - wrapper) : undefined;
        this.ifcElectricCurrentMeasure - wrapper;
        (props.ifcElectricCurrentMeasure - wrapper) ? new IfcElectricCurrentMeasure - wrapper(props.ifcElectricCurrentMeasure - wrapper) : undefined;
        this.ifcElectricResistanceMeasure - wrapper;
        (props.ifcElectricResistanceMeasure - wrapper) ? new IfcElectricResistanceMeasure - wrapper(props.ifcElectricResistanceMeasure - wrapper) : undefined;
        this.ifcElectricVoltageMeasure - wrapper;
        (props.ifcElectricVoltageMeasure - wrapper) ? new IfcElectricVoltageMeasure - wrapper(props.ifcElectricVoltageMeasure - wrapper) : undefined;
        this.ifcEnergyMeasure - wrapper;
        (props.ifcEnergyMeasure - wrapper) ? new IfcEnergyMeasure - wrapper(props.ifcEnergyMeasure - wrapper) : undefined;
        this.ifcForceMeasure - wrapper;
        (props.ifcForceMeasure - wrapper) ? new IfcForceMeasure - wrapper(props.ifcForceMeasure - wrapper) : undefined;
        this.ifcFrequencyMeasure - wrapper;
        (props.ifcFrequencyMeasure - wrapper) ? new IfcFrequencyMeasure - wrapper(props.ifcFrequencyMeasure - wrapper) : undefined;
        this.ifcHeatFluxDensityMeasure - wrapper;
        (props.ifcHeatFluxDensityMeasure - wrapper) ? new IfcHeatFluxDensityMeasure - wrapper(props.ifcHeatFluxDensityMeasure - wrapper) : undefined;
        this.ifcHeatingValueMeasure - wrapper;
        (props.ifcHeatingValueMeasure - wrapper) ? new IfcHeatingValueMeasure - wrapper(props.ifcHeatingValueMeasure - wrapper) : undefined;
        this.ifcIdentifier - wrapper;
        (props.ifcIdentifier - wrapper) ? new IfcIdentifier - wrapper(props.ifcIdentifier - wrapper) : undefined;
        this.ifcIlluminanceMeasure - wrapper;
        (props.ifcIlluminanceMeasure - wrapper) ? new IfcIlluminanceMeasure - wrapper(props.ifcIlluminanceMeasure - wrapper) : undefined;
        this.ifcInductanceMeasure - wrapper;
        (props.ifcInductanceMeasure - wrapper) ? new IfcInductanceMeasure - wrapper(props.ifcInductanceMeasure - wrapper) : undefined;
        this.ifcInteger - wrapper;
        (props.ifcInteger - wrapper) ? new IfcInteger - wrapper(props.ifcInteger - wrapper) : undefined;
        this.ifcIntegerCountRateMeasure - wrapper;
        (props.ifcIntegerCountRateMeasure - wrapper) ? new IfcIntegerCountRateMeasure - wrapper(props.ifcIntegerCountRateMeasure - wrapper) : undefined;
        this.ifcIonConcentrationMeasure - wrapper;
        (props.ifcIonConcentrationMeasure - wrapper) ? new IfcIonConcentrationMeasure - wrapper(props.ifcIonConcentrationMeasure - wrapper) : undefined;
        this.ifcIsothermalMoistureCapacityMeasure - wrapper;
        (props.ifcIsothermalMoistureCapacityMeasure - wrapper) ? new IfcIsothermalMoistureCapacityMeasure - wrapper(props.ifcIsothermalMoistureCapacityMeasure - wrapper) : undefined;
        this.ifcKinematicViscosityMeasure - wrapper;
        (props.ifcKinematicViscosityMeasure - wrapper) ? new IfcKinematicViscosityMeasure - wrapper(props.ifcKinematicViscosityMeasure - wrapper) : undefined;
        this.ifcLabel - wrapper;
        (props.ifcLabel - wrapper) ? new IfcLabel - wrapper(props.ifcLabel - wrapper) : undefined;
        this.ifcLengthMeasure - wrapper;
        (props.ifcLengthMeasure - wrapper) ? new IfcLengthMeasure - wrapper(props.ifcLengthMeasure - wrapper) : undefined;
        this.ifcLineIndex - wrapper;
        (props.ifcLineIndex - wrapper) ? new IfcLineIndex - wrapper(props.ifcLineIndex - wrapper) : undefined;
        this.ifcLinearForceMeasure - wrapper;
        (props.ifcLinearForceMeasure - wrapper) ? new IfcLinearForceMeasure - wrapper(props.ifcLinearForceMeasure - wrapper) : undefined;
        this.ifcLinearMomentMeasure - wrapper;
        (props.ifcLinearMomentMeasure - wrapper) ? new IfcLinearMomentMeasure - wrapper(props.ifcLinearMomentMeasure - wrapper) : undefined;
        this.ifcLinearStiffnessMeasure - wrapper;
        (props.ifcLinearStiffnessMeasure - wrapper) ? new IfcLinearStiffnessMeasure - wrapper(props.ifcLinearStiffnessMeasure - wrapper) : undefined;
        this.ifcLinearVelocityMeasure - wrapper;
        (props.ifcLinearVelocityMeasure - wrapper) ? new IfcLinearVelocityMeasure - wrapper(props.ifcLinearVelocityMeasure - wrapper) : undefined;
        this.ifcLogical - wrapper;
        (props.ifcLogical - wrapper) ? new IfcLogical - wrapper(props.ifcLogical - wrapper) : undefined;
        this.ifcLuminousFluxMeasure - wrapper;
        (props.ifcLuminousFluxMeasure - wrapper) ? new IfcLuminousFluxMeasure - wrapper(props.ifcLuminousFluxMeasure - wrapper) : undefined;
        this.ifcLuminousIntensityDistributionMeasure - wrapper;
        (props.ifcLuminousIntensityDistributionMeasure - wrapper) ? new IfcLuminousIntensityDistributionMeasure - wrapper(props.ifcLuminousIntensityDistributionMeasure - wrapper) : undefined;
        this.ifcLuminousIntensityMeasure - wrapper;
        (props.ifcLuminousIntensityMeasure - wrapper) ? new IfcLuminousIntensityMeasure - wrapper(props.ifcLuminousIntensityMeasure - wrapper) : undefined;
        this.ifcMagneticFluxDensityMeasure - wrapper;
        (props.ifcMagneticFluxDensityMeasure - wrapper) ? new IfcMagneticFluxDensityMeasure - wrapper(props.ifcMagneticFluxDensityMeasure - wrapper) : undefined;
        this.ifcMagneticFluxMeasure - wrapper;
        (props.ifcMagneticFluxMeasure - wrapper) ? new IfcMagneticFluxMeasure - wrapper(props.ifcMagneticFluxMeasure - wrapper) : undefined;
        this.ifcMassDensityMeasure - wrapper;
        (props.ifcMassDensityMeasure - wrapper) ? new IfcMassDensityMeasure - wrapper(props.ifcMassDensityMeasure - wrapper) : undefined;
        this.ifcMassFlowRateMeasure - wrapper;
        (props.ifcMassFlowRateMeasure - wrapper) ? new IfcMassFlowRateMeasure - wrapper(props.ifcMassFlowRateMeasure - wrapper) : undefined;
        this.ifcMassMeasure - wrapper;
        (props.ifcMassMeasure - wrapper) ? new IfcMassMeasure - wrapper(props.ifcMassMeasure - wrapper) : undefined;
        this.ifcMassPerLengthMeasure - wrapper;
        (props.ifcMassPerLengthMeasure - wrapper) ? new IfcMassPerLengthMeasure - wrapper(props.ifcMassPerLengthMeasure - wrapper) : undefined;
        this.ifcModulusOfElasticityMeasure - wrapper;
        (props.ifcModulusOfElasticityMeasure - wrapper) ? new IfcModulusOfElasticityMeasure - wrapper(props.ifcModulusOfElasticityMeasure - wrapper) : undefined;
        this.ifcModulusOfLinearSubgradeReactionMeasure - wrapper;
        (props.ifcModulusOfLinearSubgradeReactionMeasure - wrapper) ? new IfcModulusOfLinearSubgradeReactionMeasure - wrapper(props.ifcModulusOfLinearSubgradeReactionMeasure - wrapper) : undefined;
        this.ifcModulusOfRotationalSubgradeReactionMeasure - wrapper;
        (props.ifcModulusOfRotationalSubgradeReactionMeasure - wrapper) ? new IfcModulusOfRotationalSubgradeReactionMeasure - wrapper(props.ifcModulusOfRotationalSubgradeReactionMeasure - wrapper) : undefined;
        this.ifcModulusOfSubgradeReactionMeasure - wrapper;
        (props.ifcModulusOfSubgradeReactionMeasure - wrapper) ? new IfcModulusOfSubgradeReactionMeasure - wrapper(props.ifcModulusOfSubgradeReactionMeasure - wrapper) : undefined;
        this.ifcMoistureDiffusivityMeasure - wrapper;
        (props.ifcMoistureDiffusivityMeasure - wrapper) ? new IfcMoistureDiffusivityMeasure - wrapper(props.ifcMoistureDiffusivityMeasure - wrapper) : undefined;
        this.ifcMolecularWeightMeasure - wrapper;
        (props.ifcMolecularWeightMeasure - wrapper) ? new IfcMolecularWeightMeasure - wrapper(props.ifcMolecularWeightMeasure - wrapper) : undefined;
        this.ifcMomentOfInertiaMeasure - wrapper;
        (props.ifcMomentOfInertiaMeasure - wrapper) ? new IfcMomentOfInertiaMeasure - wrapper(props.ifcMomentOfInertiaMeasure - wrapper) : undefined;
        this.ifcMonetaryMeasure - wrapper;
        (props.ifcMonetaryMeasure - wrapper) ? new IfcMonetaryMeasure - wrapper(props.ifcMonetaryMeasure - wrapper) : undefined;
        this.ifcNonNegativeLengthMeasure - wrapper;
        (props.ifcNonNegativeLengthMeasure - wrapper) ? new IfcNonNegativeLengthMeasure - wrapper(props.ifcNonNegativeLengthMeasure - wrapper) : undefined;
        this.ifcNormalisedRatioMeasure - wrapper;
        (props.ifcNormalisedRatioMeasure - wrapper) ? new IfcNormalisedRatioMeasure - wrapper(props.ifcNormalisedRatioMeasure - wrapper) : undefined;
        this.ifcNullStyle - wrapper;
        (props.ifcNullStyle - wrapper) ? new IfcNullStyle - wrapper(props.ifcNullStyle - wrapper) : undefined;
        this.ifcNumericMeasure - wrapper;
        (props.ifcNumericMeasure - wrapper) ? new IfcNumericMeasure - wrapper(props.ifcNumericMeasure - wrapper) : undefined;
        this.ifcPHMeasure - wrapper;
        (props.ifcPHMeasure - wrapper) ? new IfcPHMeasure - wrapper(props.ifcPHMeasure - wrapper) : undefined;
        this.ifcParameterValue - wrapper;
        (props.ifcParameterValue - wrapper) ? new IfcParameterValue - wrapper(props.ifcParameterValue - wrapper) : undefined;
        this.ifcPlanarForceMeasure - wrapper;
        (props.ifcPlanarForceMeasure - wrapper) ? new IfcPlanarForceMeasure - wrapper(props.ifcPlanarForceMeasure - wrapper) : undefined;
        this.ifcPlaneAngleMeasure - wrapper;
        (props.ifcPlaneAngleMeasure - wrapper) ? new IfcPlaneAngleMeasure - wrapper(props.ifcPlaneAngleMeasure - wrapper) : undefined;
        this.ifcPositiveInteger - wrapper;
        (props.ifcPositiveInteger - wrapper) ? new IfcPositiveInteger - wrapper(props.ifcPositiveInteger - wrapper) : undefined;
        this.ifcPositiveLengthMeasure - wrapper;
        (props.ifcPositiveLengthMeasure - wrapper) ? new IfcPositiveLengthMeasure - wrapper(props.ifcPositiveLengthMeasure - wrapper) : undefined;
        this.ifcPositivePlaneAngleMeasure - wrapper;
        (props.ifcPositivePlaneAngleMeasure - wrapper) ? new IfcPositivePlaneAngleMeasure - wrapper(props.ifcPositivePlaneAngleMeasure - wrapper) : undefined;
        this.ifcPositiveRatioMeasure - wrapper;
        (props.ifcPositiveRatioMeasure - wrapper) ? new IfcPositiveRatioMeasure - wrapper(props.ifcPositiveRatioMeasure - wrapper) : undefined;
        this.ifcPowerMeasure - wrapper;
        (props.ifcPowerMeasure - wrapper) ? new IfcPowerMeasure - wrapper(props.ifcPowerMeasure - wrapper) : undefined;
        this.ifcPressureMeasure - wrapper;
        (props.ifcPressureMeasure - wrapper) ? new IfcPressureMeasure - wrapper(props.ifcPressureMeasure - wrapper) : undefined;
        this.ifcPropertySetDefinitionSet - wrapper;
        (props.ifcPropertySetDefinitionSet - wrapper) ? new IfcPropertySetDefinitionSet - wrapper(props.ifcPropertySetDefinitionSet - wrapper) : undefined;
        this.ifcRadioActivityMeasure - wrapper;
        (props.ifcRadioActivityMeasure - wrapper) ? new IfcRadioActivityMeasure - wrapper(props.ifcRadioActivityMeasure - wrapper) : undefined;
        this.ifcRatioMeasure - wrapper;
        (props.ifcRatioMeasure - wrapper) ? new IfcRatioMeasure - wrapper(props.ifcRatioMeasure - wrapper) : undefined;
        this.ifcReal - wrapper;
        (props.ifcReal - wrapper) ? new IfcReal - wrapper(props.ifcReal - wrapper) : undefined;
        this.ifcRotationalFrequencyMeasure - wrapper;
        (props.ifcRotationalFrequencyMeasure - wrapper) ? new IfcRotationalFrequencyMeasure - wrapper(props.ifcRotationalFrequencyMeasure - wrapper) : undefined;
        this.ifcRotationalMassMeasure - wrapper;
        (props.ifcRotationalMassMeasure - wrapper) ? new IfcRotationalMassMeasure - wrapper(props.ifcRotationalMassMeasure - wrapper) : undefined;
        this.ifcRotationalStiffnessMeasure - wrapper;
        (props.ifcRotationalStiffnessMeasure - wrapper) ? new IfcRotationalStiffnessMeasure - wrapper(props.ifcRotationalStiffnessMeasure - wrapper) : undefined;
        this.ifcSectionModulusMeasure - wrapper;
        (props.ifcSectionModulusMeasure - wrapper) ? new IfcSectionModulusMeasure - wrapper(props.ifcSectionModulusMeasure - wrapper) : undefined;
        this.ifcSectionalAreaIntegralMeasure - wrapper;
        (props.ifcSectionalAreaIntegralMeasure - wrapper) ? new IfcSectionalAreaIntegralMeasure - wrapper(props.ifcSectionalAreaIntegralMeasure - wrapper) : undefined;
        this.ifcShearModulusMeasure - wrapper;
        (props.ifcShearModulusMeasure - wrapper) ? new IfcShearModulusMeasure - wrapper(props.ifcShearModulusMeasure - wrapper) : undefined;
        this.ifcSolidAngleMeasure - wrapper;
        (props.ifcSolidAngleMeasure - wrapper) ? new IfcSolidAngleMeasure - wrapper(props.ifcSolidAngleMeasure - wrapper) : undefined;
        this.ifcSoundPowerLevelMeasure - wrapper;
        (props.ifcSoundPowerLevelMeasure - wrapper) ? new IfcSoundPowerLevelMeasure - wrapper(props.ifcSoundPowerLevelMeasure - wrapper) : undefined;
        this.ifcSoundPowerMeasure - wrapper;
        (props.ifcSoundPowerMeasure - wrapper) ? new IfcSoundPowerMeasure - wrapper(props.ifcSoundPowerMeasure - wrapper) : undefined;
        this.ifcSoundPressureLevelMeasure - wrapper;
        (props.ifcSoundPressureLevelMeasure - wrapper) ? new IfcSoundPressureLevelMeasure - wrapper(props.ifcSoundPressureLevelMeasure - wrapper) : undefined;
        this.ifcSoundPressureMeasure - wrapper;
        (props.ifcSoundPressureMeasure - wrapper) ? new IfcSoundPressureMeasure - wrapper(props.ifcSoundPressureMeasure - wrapper) : undefined;
        this.ifcSpecificHeatCapacityMeasure - wrapper;
        (props.ifcSpecificHeatCapacityMeasure - wrapper) ? new IfcSpecificHeatCapacityMeasure - wrapper(props.ifcSpecificHeatCapacityMeasure - wrapper) : undefined;
        this.ifcSpecularExponent - wrapper;
        (props.ifcSpecularExponent - wrapper) ? new IfcSpecularExponent - wrapper(props.ifcSpecularExponent - wrapper) : undefined;
        this.ifcSpecularRoughness - wrapper;
        (props.ifcSpecularRoughness - wrapper) ? new IfcSpecularRoughness - wrapper(props.ifcSpecularRoughness - wrapper) : undefined;
        this.ifcTemperatureGradientMeasure - wrapper;
        (props.ifcTemperatureGradientMeasure - wrapper) ? new IfcTemperatureGradientMeasure - wrapper(props.ifcTemperatureGradientMeasure - wrapper) : undefined;
        this.ifcTemperatureRateOfChangeMeasure - wrapper;
        (props.ifcTemperatureRateOfChangeMeasure - wrapper) ? new IfcTemperatureRateOfChangeMeasure - wrapper(props.ifcTemperatureRateOfChangeMeasure - wrapper) : undefined;
        this.ifcText - wrapper;
        (props.ifcText - wrapper) ? new IfcText - wrapper(props.ifcText - wrapper) : undefined;
        this.ifcTextFontName - wrapper;
        (props.ifcTextFontName - wrapper) ? new IfcTextFontName - wrapper(props.ifcTextFontName - wrapper) : undefined;
        this.ifcThermalAdmittanceMeasure - wrapper;
        (props.ifcThermalAdmittanceMeasure - wrapper) ? new IfcThermalAdmittanceMeasure - wrapper(props.ifcThermalAdmittanceMeasure - wrapper) : undefined;
        this.ifcThermalConductivityMeasure - wrapper;
        (props.ifcThermalConductivityMeasure - wrapper) ? new IfcThermalConductivityMeasure - wrapper(props.ifcThermalConductivityMeasure - wrapper) : undefined;
        this.ifcThermalExpansionCoefficientMeasure - wrapper;
        (props.ifcThermalExpansionCoefficientMeasure - wrapper) ? new IfcThermalExpansionCoefficientMeasure - wrapper(props.ifcThermalExpansionCoefficientMeasure - wrapper) : undefined;
        this.ifcThermalResistanceMeasure - wrapper;
        (props.ifcThermalResistanceMeasure - wrapper) ? new IfcThermalResistanceMeasure - wrapper(props.ifcThermalResistanceMeasure - wrapper) : undefined;
        this.ifcThermalTransmittanceMeasure - wrapper;
        (props.ifcThermalTransmittanceMeasure - wrapper) ? new IfcThermalTransmittanceMeasure - wrapper(props.ifcThermalTransmittanceMeasure - wrapper) : undefined;
        this.ifcThermodynamicTemperatureMeasure - wrapper;
        (props.ifcThermodynamicTemperatureMeasure - wrapper) ? new IfcThermodynamicTemperatureMeasure - wrapper(props.ifcThermodynamicTemperatureMeasure - wrapper) : undefined;
        this.ifcTime - wrapper;
        (props.ifcTime - wrapper) ? new IfcTime - wrapper(props.ifcTime - wrapper) : undefined;
        this.ifcTimeMeasure - wrapper;
        (props.ifcTimeMeasure - wrapper) ? new IfcTimeMeasure - wrapper(props.ifcTimeMeasure - wrapper) : undefined;
        this.ifcTimeStamp - wrapper;
        (props.ifcTimeStamp - wrapper) ? new IfcTimeStamp - wrapper(props.ifcTimeStamp - wrapper) : undefined;
        this.ifcTorqueMeasure - wrapper;
        (props.ifcTorqueMeasure - wrapper) ? new IfcTorqueMeasure - wrapper(props.ifcTorqueMeasure - wrapper) : undefined;
        this.ifcVaporPermeabilityMeasure - wrapper;
        (props.ifcVaporPermeabilityMeasure - wrapper) ? new IfcVaporPermeabilityMeasure - wrapper(props.ifcVaporPermeabilityMeasure - wrapper) : undefined;
        this.ifcVolumeMeasure - wrapper;
        (props.ifcVolumeMeasure - wrapper) ? new IfcVolumeMeasure - wrapper(props.ifcVolumeMeasure - wrapper) : undefined;
        this.ifcVolumetricFlowRateMeasure - wrapper;
        (props.ifcVolumetricFlowRateMeasure - wrapper) ? new IfcVolumetricFlowRateMeasure - wrapper(props.ifcVolumetricFlowRateMeasure - wrapper) : undefined;
        this.ifcWarpingConstantMeasure - wrapper;
        (props.ifcWarpingConstantMeasure - wrapper) ? new IfcWarpingConstantMeasure - wrapper(props.ifcWarpingConstantMeasure - wrapper) : undefined;
        this.ifcWarpingMomentMeasure - wrapper;
        (props.ifcWarpingMomentMeasure - wrapper) ? new IfcWarpingMomentMeasure - wrapper(props.ifcWarpingMomentMeasure - wrapper) : undefined;
    }
}
var InstanceAttributes = /** @class */ (function () {
    function InstanceAttributes(props) {
        this["@class"] = ".InstanceAttributes";
        if (props) {
            this.$id = props.$id;
            this.$path = props.$path;
            this.$pos = props.$pos;
        }
    }
    return InstanceAttributes;
}());
exports.InstanceAttributes = InstanceAttributes;
var Uos = /** @class */ (function () {
    function Uos() {
    }
    return Uos;
}());
exports.Uos = Uos;
-anyURI;
$configuration: Seq - anyURI;
constructor(props ?  : Uos);
{
    this["@class"] = ".Uos";
    if (props) {
        this.header = (props.header) ? new Header(props.header) : undefined;
        this.$id = props.$id;
        this.$express = props.$express;
        this.$configuration = props.$configuration;
    }
}
var Header = /** @class */ (function () {
    function Header(props) {
        this["@class"] = ".Header";
        if (props) {
            this.name = props.name;
            this.time_stamp = props.time_stamp;
            this.author = props.author;
            this.organization = props.organization;
            this.preprocessor_version = props.preprocessor_version;
            this.originating_system = props.originating_system;
            this.authorization = props.authorization;
            this.documentation = props.documentation;
        }
    }
    return Header;
}());
exports.Header = Header;
var IfcXML = /** @class */ (function () {
    function IfcXML(props) {
        this["@class"] = ".IfcXML";
    }
    return IfcXML;
}());
exports.IfcXML = IfcXML;
var IfcActionRequest = /** @class */ (function () {
    function IfcActionRequest(props) {
        this["@class"] = ".IfcActionRequest";
    }
    return IfcActionRequest;
}());
exports.IfcActionRequest = IfcActionRequest;
var IfcActorRole = /** @class */ (function () {
    function IfcActorRole(props) {
        this["@class"] = ".IfcActorRole";
    }
    return IfcActorRole;
}());
exports.IfcActorRole = IfcActorRole;
var IfcActuator = /** @class */ (function () {
    function IfcActuator(props) {
        this["@class"] = ".IfcActuator";
    }
    return IfcActuator;
}());
exports.IfcActuator = IfcActuator;
var IfcActuatorType = /** @class */ (function () {
    function IfcActuatorType(props) {
        this["@class"] = ".IfcActuatorType";
    }
    return IfcActuatorType;
}());
exports.IfcActuatorType = IfcActuatorType;
var IfcAddress = /** @class */ (function () {
    function IfcAddress(props) {
        this["@class"] = ".IfcAddress";
    }
    return IfcAddress;
}());
exports.IfcAddress = IfcAddress;
var IfcAdvancedBrep = /** @class */ (function () {
    function IfcAdvancedBrep(props) {
        this["@class"] = ".IfcAdvancedBrep";
    }
    return IfcAdvancedBrep;
}());
exports.IfcAdvancedBrep = IfcAdvancedBrep;
var IfcAdvancedFace = /** @class */ (function () {
    function IfcAdvancedFace(props) {
        this["@class"] = ".IfcAdvancedFace";
    }
    return IfcAdvancedFace;
}());
exports.IfcAdvancedFace = IfcAdvancedFace;
var IfcAirTerminal = /** @class */ (function () {
    function IfcAirTerminal(props) {
        this["@class"] = ".IfcAirTerminal";
    }
    return IfcAirTerminal;
}());
exports.IfcAirTerminal = IfcAirTerminal;
var IfcAirTerminalBox = /** @class */ (function () {
    function IfcAirTerminalBox(props) {
        this["@class"] = ".IfcAirTerminalBox";
    }
    return IfcAirTerminalBox;
}());
exports.IfcAirTerminalBox = IfcAirTerminalBox;
var IfcAirTerminalBoxType = /** @class */ (function () {
    function IfcAirTerminalBoxType(props) {
        this["@class"] = ".IfcAirTerminalBoxType";
    }
    return IfcAirTerminalBoxType;
}());
exports.IfcAirTerminalBoxType = IfcAirTerminalBoxType;
var IfcAirTerminalType = /** @class */ (function () {
    function IfcAirTerminalType(props) {
        this["@class"] = ".IfcAirTerminalType";
    }
    return IfcAirTerminalType;
}());
exports.IfcAirTerminalType = IfcAirTerminalType;
var IfcAirToAirHeatRecovery = /** @class */ (function () {
    function IfcAirToAirHeatRecovery(props) {
        this["@class"] = ".IfcAirToAirHeatRecovery";
    }
    return IfcAirToAirHeatRecovery;
}());
exports.IfcAirToAirHeatRecovery = IfcAirToAirHeatRecovery;
var IfcAirToAirHeatRecoveryType = /** @class */ (function () {
    function IfcAirToAirHeatRecoveryType(props) {
        this["@class"] = ".IfcAirToAirHeatRecoveryType";
    }
    return IfcAirToAirHeatRecoveryType;
}());
exports.IfcAirToAirHeatRecoveryType = IfcAirToAirHeatRecoveryType;
var IfcAlarm = /** @class */ (function () {
    function IfcAlarm(props) {
        this["@class"] = ".IfcAlarm";
    }
    return IfcAlarm;
}());
exports.IfcAlarm = IfcAlarm;
var IfcAlarmType = /** @class */ (function () {
    function IfcAlarmType(props) {
        this["@class"] = ".IfcAlarmType";
    }
    return IfcAlarmType;
}());
exports.IfcAlarmType = IfcAlarmType;
var IfcAnnotation = /** @class */ (function () {
    function IfcAnnotation(props) {
        this["@class"] = ".IfcAnnotation";
    }
    return IfcAnnotation;
}());
exports.IfcAnnotation = IfcAnnotation;
var IfcAsymmetricIShapeProfileDef = /** @class */ (function () {
    function IfcAsymmetricIShapeProfileDef(props) {
        this["@class"] = ".IfcAsymmetricIShapeProfileDef";
    }
    return IfcAsymmetricIShapeProfileDef;
}());
exports.IfcAsymmetricIShapeProfileDef = IfcAsymmetricIShapeProfileDef;
var IfcAudioVisualAppliance = /** @class */ (function () {
    function IfcAudioVisualAppliance(props) {
        this["@class"] = ".IfcAudioVisualAppliance";
    }
    return IfcAudioVisualAppliance;
}());
exports.IfcAudioVisualAppliance = IfcAudioVisualAppliance;
var IfcAudioVisualApplianceType = /** @class */ (function () {
    function IfcAudioVisualApplianceType(props) {
        this["@class"] = ".IfcAudioVisualApplianceType";
    }
    return IfcAudioVisualApplianceType;
}());
exports.IfcAudioVisualApplianceType = IfcAudioVisualApplianceType;
var IfcBSplineCurveWithKnots = /** @class */ (function () {
    function IfcBSplineCurveWithKnots(props) {
        this["@class"] = ".IfcBSplineCurveWithKnots";
    }
    return IfcBSplineCurveWithKnots;
}());
exports.IfcBSplineCurveWithKnots = IfcBSplineCurveWithKnots;
var IfcBSplineSurfaceWithKnots = /** @class */ (function () {
    function IfcBSplineSurfaceWithKnots(props) {
        this["@class"] = ".IfcBSplineSurfaceWithKnots";
    }
    return IfcBSplineSurfaceWithKnots;
}());
exports.IfcBSplineSurfaceWithKnots = IfcBSplineSurfaceWithKnots;
var IfcBeam = /** @class */ (function () {
    function IfcBeam(props) {
        this["@class"] = ".IfcBeam";
    }
    return IfcBeam;
}());
exports.IfcBeam = IfcBeam;
var IfcBeamStandardCase = /** @class */ (function () {
    function IfcBeamStandardCase(props) {
        this["@class"] = ".IfcBeamStandardCase";
    }
    return IfcBeamStandardCase;
}());
exports.IfcBeamStandardCase = IfcBeamStandardCase;
var IfcBeamType = /** @class */ (function () {
    function IfcBeamType(props) {
        this["@class"] = ".IfcBeamType";
    }
    return IfcBeamType;
}());
exports.IfcBeamType = IfcBeamType;
var IfcBlock = /** @class */ (function () {
    function IfcBlock(props) {
        this["@class"] = ".IfcBlock";
    }
    return IfcBlock;
}());
exports.IfcBlock = IfcBlock;
var IfcBoiler = /** @class */ (function () {
    function IfcBoiler(props) {
        this["@class"] = ".IfcBoiler";
    }
    return IfcBoiler;
}());
exports.IfcBoiler = IfcBoiler;
var IfcBoilerType = /** @class */ (function () {
    function IfcBoilerType(props) {
        this["@class"] = ".IfcBoilerType";
    }
    return IfcBoilerType;
}());
exports.IfcBoilerType = IfcBoilerType;
var IfcBooleanClippingResult = /** @class */ (function () {
    function IfcBooleanClippingResult(props) {
        this["@class"] = ".IfcBooleanClippingResult";
    }
    return IfcBooleanClippingResult;
}());
exports.IfcBooleanClippingResult = IfcBooleanClippingResult;
var IfcBoundaryCondition = /** @class */ (function () {
    function IfcBoundaryCondition(props) {
        this["@class"] = ".IfcBoundaryCondition";
    }
    return IfcBoundaryCondition;
}());
exports.IfcBoundaryCondition = IfcBoundaryCondition;
var IfcBoundaryCurve = /** @class */ (function () {
    function IfcBoundaryCurve(props) {
        this["@class"] = ".IfcBoundaryCurve";
    }
    return IfcBoundaryCurve;
}());
exports.IfcBoundaryCurve = IfcBoundaryCurve;
var IfcBoundedCurve = /** @class */ (function () {
    function IfcBoundedCurve(props) {
        this["@class"] = ".IfcBoundedCurve";
    }
    return IfcBoundedCurve;
}());
exports.IfcBoundedCurve = IfcBoundedCurve;
var IfcBoundedSurface = /** @class */ (function () {
    function IfcBoundedSurface(props) {
        this["@class"] = ".IfcBoundedSurface";
    }
    return IfcBoundedSurface;
}());
exports.IfcBoundedSurface = IfcBoundedSurface;
var IfcBuildingElement = /** @class */ (function () {
    function IfcBuildingElement(props) {
        this["@class"] = ".IfcBuildingElement";
    }
    return IfcBuildingElement;
}());
exports.IfcBuildingElement = IfcBuildingElement;
var IfcBuildingElementPart = /** @class */ (function () {
    function IfcBuildingElementPart(props) {
        this["@class"] = ".IfcBuildingElementPart";
    }
    return IfcBuildingElementPart;
}());
exports.IfcBuildingElementPart = IfcBuildingElementPart;
var IfcBuildingElementPartType = /** @class */ (function () {
    function IfcBuildingElementPartType(props) {
        this["@class"] = ".IfcBuildingElementPartType";
    }
    return IfcBuildingElementPartType;
}());
exports.IfcBuildingElementPartType = IfcBuildingElementPartType;
var IfcBuildingElementProxy = /** @class */ (function () {
    function IfcBuildingElementProxy(props) {
        this["@class"] = ".IfcBuildingElementProxy";
    }
    return IfcBuildingElementProxy;
}());
exports.IfcBuildingElementProxy = IfcBuildingElementProxy;
var IfcBuildingElementProxyType = /** @class */ (function () {
    function IfcBuildingElementProxyType(props) {
        this["@class"] = ".IfcBuildingElementProxyType";
    }
    return IfcBuildingElementProxyType;
}());
exports.IfcBuildingElementProxyType = IfcBuildingElementProxyType;
var IfcBuildingElementType = /** @class */ (function () {
    function IfcBuildingElementType(props) {
        this["@class"] = ".IfcBuildingElementType";
    }
    return IfcBuildingElementType;
}());
exports.IfcBuildingElementType = IfcBuildingElementType;
var IfcBuildingStorey = /** @class */ (function () {
    function IfcBuildingStorey(props) {
        this["@class"] = ".IfcBuildingStorey";
    }
    return IfcBuildingStorey;
}());
exports.IfcBuildingStorey = IfcBuildingStorey;
var IfcBuildingSystem = /** @class */ (function () {
    function IfcBuildingSystem(props) {
        this["@class"] = ".IfcBuildingSystem";
    }
    return IfcBuildingSystem;
}());
exports.IfcBuildingSystem = IfcBuildingSystem;
var IfcBurner = /** @class */ (function () {
    function IfcBurner(props) {
        this["@class"] = ".IfcBurner";
    }
    return IfcBurner;
}());
exports.IfcBurner = IfcBurner;
var IfcBurnerType = /** @class */ (function () {
    function IfcBurnerType(props) {
        this["@class"] = ".IfcBurnerType";
    }
    return IfcBurnerType;
}());
exports.IfcBurnerType = IfcBurnerType;
var IfcCShapeProfileDef = /** @class */ (function () {
    function IfcCShapeProfileDef(props) {
        this["@class"] = ".IfcCShapeProfileDef";
    }
    return IfcCShapeProfileDef;
}());
exports.IfcCShapeProfileDef = IfcCShapeProfileDef;
var IfcCableCarrierFitting = /** @class */ (function () {
    function IfcCableCarrierFitting(props) {
        this["@class"] = ".IfcCableCarrierFitting";
    }
    return IfcCableCarrierFitting;
}());
exports.IfcCableCarrierFitting = IfcCableCarrierFitting;
var IfcCableCarrierFittingType = /** @class */ (function () {
    function IfcCableCarrierFittingType(props) {
        this["@class"] = ".IfcCableCarrierFittingType";
    }
    return IfcCableCarrierFittingType;
}());
exports.IfcCableCarrierFittingType = IfcCableCarrierFittingType;
var IfcCableCarrierSegment = /** @class */ (function () {
    function IfcCableCarrierSegment(props) {
        this["@class"] = ".IfcCableCarrierSegment";
    }
    return IfcCableCarrierSegment;
}());
exports.IfcCableCarrierSegment = IfcCableCarrierSegment;
var IfcCableCarrierSegmentType = /** @class */ (function () {
    function IfcCableCarrierSegmentType(props) {
        this["@class"] = ".IfcCableCarrierSegmentType";
    }
    return IfcCableCarrierSegmentType;
}());
exports.IfcCableCarrierSegmentType = IfcCableCarrierSegmentType;
var IfcCableFitting = /** @class */ (function () {
    function IfcCableFitting(props) {
        this["@class"] = ".IfcCableFitting";
    }
    return IfcCableFitting;
}());
exports.IfcCableFitting = IfcCableFitting;
var IfcCableFittingType = /** @class */ (function () {
    function IfcCableFittingType(props) {
        this["@class"] = ".IfcCableFittingType";
    }
    return IfcCableFittingType;
}());
exports.IfcCableFittingType = IfcCableFittingType;
var IfcCableSegment = /** @class */ (function () {
    function IfcCableSegment(props) {
        this["@class"] = ".IfcCableSegment";
    }
    return IfcCableSegment;
}());
exports.IfcCableSegment = IfcCableSegment;
var IfcCableSegmentType = /** @class */ (function () {
    function IfcCableSegmentType(props) {
        this["@class"] = ".IfcCableSegmentType";
    }
    return IfcCableSegmentType;
}());
exports.IfcCableSegmentType = IfcCableSegmentType;
var IfcCartesianPoint = /** @class */ (function () {
    function IfcCartesianPoint(props) {
        this["@class"] = ".IfcCartesianPoint";
    }
    return IfcCartesianPoint;
}());
exports.IfcCartesianPoint = IfcCartesianPoint;
var IfcCartesianPointList = /** @class */ (function () {
    function IfcCartesianPointList(props) {
        this["@class"] = ".IfcCartesianPointList";
    }
    return IfcCartesianPointList;
}());
exports.IfcCartesianPointList = IfcCartesianPointList;
var IfcCartesianPointList2D = /** @class */ (function () {
    function IfcCartesianPointList2D(props) {
        this["@class"] = ".IfcCartesianPointList2D";
    }
    return IfcCartesianPointList2D;
}());
exports.IfcCartesianPointList2D = IfcCartesianPointList2D;
var IfcCartesianPointList3D = /** @class */ (function () {
    function IfcCartesianPointList3D(props) {
        this["@class"] = ".IfcCartesianPointList3D";
    }
    return IfcCartesianPointList3D;
}());
exports.IfcCartesianPointList3D = IfcCartesianPointList3D;
var IfcCartesianTransformationOperator2D = /** @class */ (function () {
    function IfcCartesianTransformationOperator2D(props) {
        this["@class"] = ".IfcCartesianTransformationOperator2D";
    }
    return IfcCartesianTransformationOperator2D;
}());
exports.IfcCartesianTransformationOperator2D = IfcCartesianTransformationOperator2D;
var IfcCartesianTransformationOperator2DnonUniform = /** @class */ (function () {
    function IfcCartesianTransformationOperator2DnonUniform(props) {
        this["@class"] = ".IfcCartesianTransformationOperator2DnonUniform";
    }
    return IfcCartesianTransformationOperator2DnonUniform;
}());
exports.IfcCartesianTransformationOperator2DnonUniform = IfcCartesianTransformationOperator2DnonUniform;
var IfcCartesianTransformationOperator3DnonUniform = /** @class */ (function () {
    function IfcCartesianTransformationOperator3DnonUniform(props) {
        this["@class"] = ".IfcCartesianTransformationOperator3DnonUniform";
    }
    return IfcCartesianTransformationOperator3DnonUniform;
}());
exports.IfcCartesianTransformationOperator3DnonUniform = IfcCartesianTransformationOperator3DnonUniform;
var IfcCenterLineProfileDef = /** @class */ (function () {
    function IfcCenterLineProfileDef(props) {
        this["@class"] = ".IfcCenterLineProfileDef";
    }
    return IfcCenterLineProfileDef;
}());
exports.IfcCenterLineProfileDef = IfcCenterLineProfileDef;
var IfcChiller = /** @class */ (function () {
    function IfcChiller(props) {
        this["@class"] = ".IfcChiller";
    }
    return IfcChiller;
}());
exports.IfcChiller = IfcChiller;
var IfcChillerType = /** @class */ (function () {
    function IfcChillerType(props) {
        this["@class"] = ".IfcChillerType";
    }
    return IfcChillerType;
}());
exports.IfcChillerType = IfcChillerType;
var IfcChimney = /** @class */ (function () {
    function IfcChimney(props) {
        this["@class"] = ".IfcChimney";
    }
    return IfcChimney;
}());
exports.IfcChimney = IfcChimney;
var IfcChimneyType = /** @class */ (function () {
    function IfcChimneyType(props) {
        this["@class"] = ".IfcChimneyType";
    }
    return IfcChimneyType;
}());
exports.IfcChimneyType = IfcChimneyType;
var IfcCircle = /** @class */ (function () {
    function IfcCircle(props) {
        this["@class"] = ".IfcCircle";
    }
    return IfcCircle;
}());
exports.IfcCircle = IfcCircle;
var IfcCircleHollowProfileDef = /** @class */ (function () {
    function IfcCircleHollowProfileDef(props) {
        this["@class"] = ".IfcCircleHollowProfileDef";
    }
    return IfcCircleHollowProfileDef;
}());
exports.IfcCircleHollowProfileDef = IfcCircleHollowProfileDef;
var IfcCircleProfileDef = /** @class */ (function () {
    function IfcCircleProfileDef(props) {
        this["@class"] = ".IfcCircleProfileDef";
    }
    return IfcCircleProfileDef;
}());
exports.IfcCircleProfileDef = IfcCircleProfileDef;
var IfcCivilElement = /** @class */ (function () {
    function IfcCivilElement(props) {
        this["@class"] = ".IfcCivilElement";
    }
    return IfcCivilElement;
}());
exports.IfcCivilElement = IfcCivilElement;
var IfcCivilElementType = /** @class */ (function () {
    function IfcCivilElementType(props) {
        this["@class"] = ".IfcCivilElementType";
    }
    return IfcCivilElementType;
}());
exports.IfcCivilElementType = IfcCivilElementType;
var IfcClosedShell = /** @class */ (function () {
    function IfcClosedShell(props) {
        this["@class"] = ".IfcClosedShell";
    }
    return IfcClosedShell;
}());
exports.IfcClosedShell = IfcClosedShell;
var IfcCoil = /** @class */ (function () {
    function IfcCoil(props) {
        this["@class"] = ".IfcCoil";
    }
    return IfcCoil;
}());
exports.IfcCoil = IfcCoil;
var IfcCoilType = /** @class */ (function () {
    function IfcCoilType(props) {
        this["@class"] = ".IfcCoilType";
    }
    return IfcCoilType;
}());
exports.IfcCoilType = IfcCoilType;
var IfcColourRgb = /** @class */ (function () {
    function IfcColourRgb(props) {
        this["@class"] = ".IfcColourRgb";
    }
    return IfcColourRgb;
}());
exports.IfcColourRgb = IfcColourRgb;
var IfcColourRgbList = /** @class */ (function () {
    function IfcColourRgbList(props) {
        this["@class"] = ".IfcColourRgbList";
    }
    return IfcColourRgbList;
}());
exports.IfcColourRgbList = IfcColourRgbList;
var IfcColourSpecification = /** @class */ (function () {
    function IfcColourSpecification(props) {
        this["@class"] = ".IfcColourSpecification";
    }
    return IfcColourSpecification;
}());
exports.IfcColourSpecification = IfcColourSpecification;
var IfcColumn = /** @class */ (function () {
    function IfcColumn(props) {
        this["@class"] = ".IfcColumn";
    }
    return IfcColumn;
}());
exports.IfcColumn = IfcColumn;
var IfcColumnStandardCase = /** @class */ (function () {
    function IfcColumnStandardCase(props) {
        this["@class"] = ".IfcColumnStandardCase";
    }
    return IfcColumnStandardCase;
}());
exports.IfcColumnStandardCase = IfcColumnStandardCase;
var IfcColumnType = /** @class */ (function () {
    function IfcColumnType(props) {
        this["@class"] = ".IfcColumnType";
    }
    return IfcColumnType;
}());
exports.IfcColumnType = IfcColumnType;
var IfcCommunicationsAppliance = /** @class */ (function () {
    function IfcCommunicationsAppliance(props) {
        this["@class"] = ".IfcCommunicationsAppliance";
    }
    return IfcCommunicationsAppliance;
}());
exports.IfcCommunicationsAppliance = IfcCommunicationsAppliance;
var IfcCommunicationsApplianceType = /** @class */ (function () {
    function IfcCommunicationsApplianceType(props) {
        this["@class"] = ".IfcCommunicationsApplianceType";
    }
    return IfcCommunicationsApplianceType;
}());
exports.IfcCommunicationsApplianceType = IfcCommunicationsApplianceType;
var IfcCompositeCurveOnSurface = /** @class */ (function () {
    function IfcCompositeCurveOnSurface(props) {
        this["@class"] = ".IfcCompositeCurveOnSurface";
    }
    return IfcCompositeCurveOnSurface;
}());
exports.IfcCompositeCurveOnSurface = IfcCompositeCurveOnSurface;
var IfcCompressor = /** @class */ (function () {
    function IfcCompressor(props) {
        this["@class"] = ".IfcCompressor";
    }
    return IfcCompressor;
}());
exports.IfcCompressor = IfcCompressor;
var IfcCompressorType = /** @class */ (function () {
    function IfcCompressorType(props) {
        this["@class"] = ".IfcCompressorType";
    }
    return IfcCompressorType;
}());
exports.IfcCompressorType = IfcCompressorType;
var IfcCondenser = /** @class */ (function () {
    function IfcCondenser(props) {
        this["@class"] = ".IfcCondenser";
    }
    return IfcCondenser;
}());
exports.IfcCondenser = IfcCondenser;
var IfcCondenserType = /** @class */ (function () {
    function IfcCondenserType(props) {
        this["@class"] = ".IfcCondenserType";
    }
    return IfcCondenserType;
}());
exports.IfcCondenserType = IfcCondenserType;
var IfcConnectionGeometry = /** @class */ (function () {
    function IfcConnectionGeometry(props) {
        this["@class"] = ".IfcConnectionGeometry";
    }
    return IfcConnectionGeometry;
}());
exports.IfcConnectionGeometry = IfcConnectionGeometry;
var IfcConnectionPointEccentricity = /** @class */ (function () {
    function IfcConnectionPointEccentricity(props) {
        this["@class"] = ".IfcConnectionPointEccentricity";
    }
    return IfcConnectionPointEccentricity;
}());
exports.IfcConnectionPointEccentricity = IfcConnectionPointEccentricity;
var IfcConstructionEquipmentResource = /** @class */ (function () {
    function IfcConstructionEquipmentResource(props) {
        this["@class"] = ".IfcConstructionEquipmentResource";
    }
    return IfcConstructionEquipmentResource;
}());
exports.IfcConstructionEquipmentResource = IfcConstructionEquipmentResource;
var IfcConstructionEquipmentResourceType = /** @class */ (function () {
    function IfcConstructionEquipmentResourceType(props) {
        this["@class"] = ".IfcConstructionEquipmentResourceType";
    }
    return IfcConstructionEquipmentResourceType;
}());
exports.IfcConstructionEquipmentResourceType = IfcConstructionEquipmentResourceType;
var IfcConstructionMaterialResource = /** @class */ (function () {
    function IfcConstructionMaterialResource(props) {
        this["@class"] = ".IfcConstructionMaterialResource";
    }
    return IfcConstructionMaterialResource;
}());
exports.IfcConstructionMaterialResource = IfcConstructionMaterialResource;
var IfcConstructionMaterialResourceType = /** @class */ (function () {
    function IfcConstructionMaterialResourceType(props) {
        this["@class"] = ".IfcConstructionMaterialResourceType";
    }
    return IfcConstructionMaterialResourceType;
}());
exports.IfcConstructionMaterialResourceType = IfcConstructionMaterialResourceType;
var IfcConstructionProductResource = /** @class */ (function () {
    function IfcConstructionProductResource(props) {
        this["@class"] = ".IfcConstructionProductResource";
    }
    return IfcConstructionProductResource;
}());
exports.IfcConstructionProductResource = IfcConstructionProductResource;
var IfcConstructionProductResourceType = /** @class */ (function () {
    function IfcConstructionProductResourceType(props) {
        this["@class"] = ".IfcConstructionProductResourceType";
    }
    return IfcConstructionProductResourceType;
}());
exports.IfcConstructionProductResourceType = IfcConstructionProductResourceType;
var IfcContextDependentUnit = /** @class */ (function () {
    function IfcContextDependentUnit(props) {
        this["@class"] = ".IfcContextDependentUnit";
    }
    return IfcContextDependentUnit;
}());
exports.IfcContextDependentUnit = IfcContextDependentUnit;
var IfcControl = /** @class */ (function () {
    function IfcControl(props) {
        this["@class"] = ".IfcControl";
    }
    return IfcControl;
}());
exports.IfcControl = IfcControl;
var IfcController = /** @class */ (function () {
    function IfcController(props) {
        this["@class"] = ".IfcController";
    }
    return IfcController;
}());
exports.IfcController = IfcController;
var IfcControllerType = /** @class */ (function () {
    function IfcControllerType(props) {
        this["@class"] = ".IfcControllerType";
    }
    return IfcControllerType;
}());
exports.IfcControllerType = IfcControllerType;
var IfcConversionBasedUnitWithOffset = /** @class */ (function () {
    function IfcConversionBasedUnitWithOffset(props) {
        this["@class"] = ".IfcConversionBasedUnitWithOffset";
    }
    return IfcConversionBasedUnitWithOffset;
}());
exports.IfcConversionBasedUnitWithOffset = IfcConversionBasedUnitWithOffset;
var IfcCooledBeam = /** @class */ (function () {
    function IfcCooledBeam(props) {
        this["@class"] = ".IfcCooledBeam";
    }
    return IfcCooledBeam;
}());
exports.IfcCooledBeam = IfcCooledBeam;
var IfcCooledBeamType = /** @class */ (function () {
    function IfcCooledBeamType(props) {
        this["@class"] = ".IfcCooledBeamType";
    }
    return IfcCooledBeamType;
}());
exports.IfcCooledBeamType = IfcCooledBeamType;
var IfcCoolingTower = /** @class */ (function () {
    function IfcCoolingTower(props) {
        this["@class"] = ".IfcCoolingTower";
    }
    return IfcCoolingTower;
}());
exports.IfcCoolingTower = IfcCoolingTower;
var IfcCoolingTowerType = /** @class */ (function () {
    function IfcCoolingTowerType(props) {
        this["@class"] = ".IfcCoolingTowerType";
    }
    return IfcCoolingTowerType;
}());
exports.IfcCoolingTowerType = IfcCoolingTowerType;
var IfcCoordinateReferenceSystem = /** @class */ (function () {
    function IfcCoordinateReferenceSystem(props) {
        this["@class"] = ".IfcCoordinateReferenceSystem";
    }
    return IfcCoordinateReferenceSystem;
}());
exports.IfcCoordinateReferenceSystem = IfcCoordinateReferenceSystem;
var IfcCostSchedule = /** @class */ (function () {
    function IfcCostSchedule(props) {
        this["@class"] = ".IfcCostSchedule";
    }
    return IfcCostSchedule;
}());
exports.IfcCostSchedule = IfcCostSchedule;
var IfcCostValue = /** @class */ (function () {
    function IfcCostValue(props) {
        this["@class"] = ".IfcCostValue";
    }
    return IfcCostValue;
}());
exports.IfcCostValue = IfcCostValue;
var IfcCovering = /** @class */ (function () {
    function IfcCovering(props) {
        this["@class"] = ".IfcCovering";
    }
    return IfcCovering;
}());
exports.IfcCovering = IfcCovering;
var IfcCoveringType = /** @class */ (function () {
    function IfcCoveringType(props) {
        this["@class"] = ".IfcCoveringType";
    }
    return IfcCoveringType;
}());
exports.IfcCoveringType = IfcCoveringType;
var IfcCrewResource = /** @class */ (function () {
    function IfcCrewResource(props) {
        this["@class"] = ".IfcCrewResource";
    }
    return IfcCrewResource;
}());
exports.IfcCrewResource = IfcCrewResource;
var IfcCrewResourceType = /** @class */ (function () {
    function IfcCrewResourceType(props) {
        this["@class"] = ".IfcCrewResourceType";
    }
    return IfcCrewResourceType;
}());
exports.IfcCrewResourceType = IfcCrewResourceType;
var IfcCurtainWall = /** @class */ (function () {
    function IfcCurtainWall(props) {
        this["@class"] = ".IfcCurtainWall";
    }
    return IfcCurtainWall;
}());
exports.IfcCurtainWall = IfcCurtainWall;
var IfcCurtainWallType = /** @class */ (function () {
    function IfcCurtainWallType(props) {
        this["@class"] = ".IfcCurtainWallType";
    }
    return IfcCurtainWallType;
}());
exports.IfcCurtainWallType = IfcCurtainWallType;
var IfcCurve = /** @class */ (function () {
    function IfcCurve(props) {
        this["@class"] = ".IfcCurve";
    }
    return IfcCurve;
}());
exports.IfcCurve = IfcCurve;
var IfcCurveStyleFontPattern = /** @class */ (function () {
    function IfcCurveStyleFontPattern(props) {
        this["@class"] = ".IfcCurveStyleFontPattern";
    }
    return IfcCurveStyleFontPattern;
}());
exports.IfcCurveStyleFontPattern = IfcCurveStyleFontPattern;
var IfcCylindricalSurface = /** @class */ (function () {
    function IfcCylindricalSurface(props) {
        this["@class"] = ".IfcCylindricalSurface";
    }
    return IfcCylindricalSurface;
}());
exports.IfcCylindricalSurface = IfcCylindricalSurface;
var IfcDamper = /** @class */ (function () {
    function IfcDamper(props) {
        this["@class"] = ".IfcDamper";
    }
    return IfcDamper;
}());
exports.IfcDamper = IfcDamper;
var IfcDamperType = /** @class */ (function () {
    function IfcDamperType(props) {
        this["@class"] = ".IfcDamperType";
    }
    return IfcDamperType;
}());
exports.IfcDamperType = IfcDamperType;
var IfcDimensionalExponents = /** @class */ (function () {
    function IfcDimensionalExponents(props) {
        this["@class"] = ".IfcDimensionalExponents";
    }
    return IfcDimensionalExponents;
}());
exports.IfcDimensionalExponents = IfcDimensionalExponents;
var IfcDirection = /** @class */ (function () {
    function IfcDirection(props) {
        this["@class"] = ".IfcDirection";
    }
    return IfcDirection;
}());
exports.IfcDirection = IfcDirection;
var IfcDiscreteAccessory = /** @class */ (function () {
    function IfcDiscreteAccessory(props) {
        this["@class"] = ".IfcDiscreteAccessory";
    }
    return IfcDiscreteAccessory;
}());
exports.IfcDiscreteAccessory = IfcDiscreteAccessory;
var IfcDiscreteAccessoryType = /** @class */ (function () {
    function IfcDiscreteAccessoryType(props) {
        this["@class"] = ".IfcDiscreteAccessoryType";
    }
    return IfcDiscreteAccessoryType;
}());
exports.IfcDiscreteAccessoryType = IfcDiscreteAccessoryType;
var IfcDistributionChamberElement = /** @class */ (function () {
    function IfcDistributionChamberElement(props) {
        this["@class"] = ".IfcDistributionChamberElement";
    }
    return IfcDistributionChamberElement;
}());
exports.IfcDistributionChamberElement = IfcDistributionChamberElement;
var IfcDistributionChamberElementType = /** @class */ (function () {
    function IfcDistributionChamberElementType(props) {
        this["@class"] = ".IfcDistributionChamberElementType";
    }
    return IfcDistributionChamberElementType;
}());
exports.IfcDistributionChamberElementType = IfcDistributionChamberElementType;
var IfcDistributionCircuit = /** @class */ (function () {
    function IfcDistributionCircuit(props) {
        this["@class"] = ".IfcDistributionCircuit";
    }
    return IfcDistributionCircuit;
}());
exports.IfcDistributionCircuit = IfcDistributionCircuit;
var IfcDistributionControlElement = /** @class */ (function () {
    function IfcDistributionControlElement(props) {
        this["@class"] = ".IfcDistributionControlElement";
    }
    return IfcDistributionControlElement;
}());
exports.IfcDistributionControlElement = IfcDistributionControlElement;
var IfcDistributionControlElementType = /** @class */ (function () {
    function IfcDistributionControlElementType(props) {
        this["@class"] = ".IfcDistributionControlElementType";
    }
    return IfcDistributionControlElementType;
}());
exports.IfcDistributionControlElementType = IfcDistributionControlElementType;
var IfcDistributionElement = /** @class */ (function () {
    function IfcDistributionElement(props) {
        this["@class"] = ".IfcDistributionElement";
    }
    return IfcDistributionElement;
}());
exports.IfcDistributionElement = IfcDistributionElement;
var IfcDistributionElementType = /** @class */ (function () {
    function IfcDistributionElementType(props) {
        this["@class"] = ".IfcDistributionElementType";
    }
    return IfcDistributionElementType;
}());
exports.IfcDistributionElementType = IfcDistributionElementType;
var IfcDistributionFlowElement = /** @class */ (function () {
    function IfcDistributionFlowElement(props) {
        this["@class"] = ".IfcDistributionFlowElement";
    }
    return IfcDistributionFlowElement;
}());
exports.IfcDistributionFlowElement = IfcDistributionFlowElement;
var IfcDistributionFlowElementType = /** @class */ (function () {
    function IfcDistributionFlowElementType(props) {
        this["@class"] = ".IfcDistributionFlowElementType";
    }
    return IfcDistributionFlowElementType;
}());
exports.IfcDistributionFlowElementType = IfcDistributionFlowElementType;
var IfcDistributionPort = /** @class */ (function () {
    function IfcDistributionPort(props) {
        this["@class"] = ".IfcDistributionPort";
    }
    return IfcDistributionPort;
}());
exports.IfcDistributionPort = IfcDistributionPort;
var IfcDistributionSystem = /** @class */ (function () {
    function IfcDistributionSystem(props) {
        this["@class"] = ".IfcDistributionSystem";
    }
    return IfcDistributionSystem;
}());
exports.IfcDistributionSystem = IfcDistributionSystem;
var IfcDoor = /** @class */ (function () {
    function IfcDoor(props) {
        this["@class"] = ".IfcDoor";
    }
    return IfcDoor;
}());
exports.IfcDoor = IfcDoor;
var IfcDoorStandardCase = /** @class */ (function () {
    function IfcDoorStandardCase(props) {
        this["@class"] = ".IfcDoorStandardCase";
    }
    return IfcDoorStandardCase;
}());
exports.IfcDoorStandardCase = IfcDoorStandardCase;
var IfcDoorStyle = /** @class */ (function () {
    function IfcDoorStyle(props) {
        this["@class"] = ".IfcDoorStyle";
    }
    return IfcDoorStyle;
}());
exports.IfcDoorStyle = IfcDoorStyle;
var IfcDoorType = /** @class */ (function () {
    function IfcDoorType(props) {
        this["@class"] = ".IfcDoorType";
    }
    return IfcDoorType;
}());
exports.IfcDoorType = IfcDoorType;
var IfcDraughtingPreDefinedColour = /** @class */ (function () {
    function IfcDraughtingPreDefinedColour(props) {
        this["@class"] = ".IfcDraughtingPreDefinedColour";
    }
    return IfcDraughtingPreDefinedColour;
}());
exports.IfcDraughtingPreDefinedColour = IfcDraughtingPreDefinedColour;
var IfcDraughtingPreDefinedCurveFont = /** @class */ (function () {
    function IfcDraughtingPreDefinedCurveFont(props) {
        this["@class"] = ".IfcDraughtingPreDefinedCurveFont";
    }
    return IfcDraughtingPreDefinedCurveFont;
}());
exports.IfcDraughtingPreDefinedCurveFont = IfcDraughtingPreDefinedCurveFont;
var IfcDuctFitting = /** @class */ (function () {
    function IfcDuctFitting(props) {
        this["@class"] = ".IfcDuctFitting";
    }
    return IfcDuctFitting;
}());
exports.IfcDuctFitting = IfcDuctFitting;
var IfcDuctFittingType = /** @class */ (function () {
    function IfcDuctFittingType(props) {
        this["@class"] = ".IfcDuctFittingType";
    }
    return IfcDuctFittingType;
}());
exports.IfcDuctFittingType = IfcDuctFittingType;
var IfcDuctSegment = /** @class */ (function () {
    function IfcDuctSegment(props) {
        this["@class"] = ".IfcDuctSegment";
    }
    return IfcDuctSegment;
}());
exports.IfcDuctSegment = IfcDuctSegment;
var IfcDuctSegmentType = /** @class */ (function () {
    function IfcDuctSegmentType(props) {
        this["@class"] = ".IfcDuctSegmentType";
    }
    return IfcDuctSegmentType;
}());
exports.IfcDuctSegmentType = IfcDuctSegmentType;
var IfcDuctSilencer = /** @class */ (function () {
    function IfcDuctSilencer(props) {
        this["@class"] = ".IfcDuctSilencer";
    }
    return IfcDuctSilencer;
}());
exports.IfcDuctSilencer = IfcDuctSilencer;
var IfcDuctSilencerType = /** @class */ (function () {
    function IfcDuctSilencerType(props) {
        this["@class"] = ".IfcDuctSilencerType";
    }
    return IfcDuctSilencerType;
}());
exports.IfcDuctSilencerType = IfcDuctSilencerType;
var IfcElectricAppliance = /** @class */ (function () {
    function IfcElectricAppliance(props) {
        this["@class"] = ".IfcElectricAppliance";
    }
    return IfcElectricAppliance;
}());
exports.IfcElectricAppliance = IfcElectricAppliance;
var IfcElectricApplianceType = /** @class */ (function () {
    function IfcElectricApplianceType(props) {
        this["@class"] = ".IfcElectricApplianceType";
    }
    return IfcElectricApplianceType;
}());
exports.IfcElectricApplianceType = IfcElectricApplianceType;
var IfcElectricDistributionBoard = /** @class */ (function () {
    function IfcElectricDistributionBoard(props) {
        this["@class"] = ".IfcElectricDistributionBoard";
    }
    return IfcElectricDistributionBoard;
}());
exports.IfcElectricDistributionBoard = IfcElectricDistributionBoard;
var IfcElectricDistributionBoardType = /** @class */ (function () {
    function IfcElectricDistributionBoardType(props) {
        this["@class"] = ".IfcElectricDistributionBoardType";
    }
    return IfcElectricDistributionBoardType;
}());
exports.IfcElectricDistributionBoardType = IfcElectricDistributionBoardType;
var IfcElectricFlowStorageDevice = /** @class */ (function () {
    function IfcElectricFlowStorageDevice(props) {
        this["@class"] = ".IfcElectricFlowStorageDevice";
    }
    return IfcElectricFlowStorageDevice;
}());
exports.IfcElectricFlowStorageDevice = IfcElectricFlowStorageDevice;
var IfcElectricFlowStorageDeviceType = /** @class */ (function () {
    function IfcElectricFlowStorageDeviceType(props) {
        this["@class"] = ".IfcElectricFlowStorageDeviceType";
    }
    return IfcElectricFlowStorageDeviceType;
}());
exports.IfcElectricFlowStorageDeviceType = IfcElectricFlowStorageDeviceType;
var IfcElectricGenerator = /** @class */ (function () {
    function IfcElectricGenerator(props) {
        this["@class"] = ".IfcElectricGenerator";
    }
    return IfcElectricGenerator;
}());
exports.IfcElectricGenerator = IfcElectricGenerator;
var IfcElectricGeneratorType = /** @class */ (function () {
    function IfcElectricGeneratorType(props) {
        this["@class"] = ".IfcElectricGeneratorType";
    }
    return IfcElectricGeneratorType;
}());
exports.IfcElectricGeneratorType = IfcElectricGeneratorType;
var IfcElectricMotor = /** @class */ (function () {
    function IfcElectricMotor(props) {
        this["@class"] = ".IfcElectricMotor";
    }
    return IfcElectricMotor;
}());
exports.IfcElectricMotor = IfcElectricMotor;
var IfcElectricMotorType = /** @class */ (function () {
    function IfcElectricMotorType(props) {
        this["@class"] = ".IfcElectricMotorType";
    }
    return IfcElectricMotorType;
}());
exports.IfcElectricMotorType = IfcElectricMotorType;
var IfcElectricTimeControl = /** @class */ (function () {
    function IfcElectricTimeControl(props) {
        this["@class"] = ".IfcElectricTimeControl";
    }
    return IfcElectricTimeControl;
}());
exports.IfcElectricTimeControl = IfcElectricTimeControl;
var IfcElectricTimeControlType = /** @class */ (function () {
    function IfcElectricTimeControlType(props) {
        this["@class"] = ".IfcElectricTimeControlType";
    }
    return IfcElectricTimeControlType;
}());
exports.IfcElectricTimeControlType = IfcElectricTimeControlType;
var IfcElementAssembly = /** @class */ (function () {
    function IfcElementAssembly(props) {
        this["@class"] = ".IfcElementAssembly";
    }
    return IfcElementAssembly;
}());
exports.IfcElementAssembly = IfcElementAssembly;
var IfcElementAssemblyType = /** @class */ (function () {
    function IfcElementAssemblyType(props) {
        this["@class"] = ".IfcElementAssemblyType";
    }
    return IfcElementAssemblyType;
}());
exports.IfcElementAssemblyType = IfcElementAssemblyType;
var IfcElementComponent = /** @class */ (function () {
    function IfcElementComponent(props) {
        this["@class"] = ".IfcElementComponent";
    }
    return IfcElementComponent;
}());
exports.IfcElementComponent = IfcElementComponent;
var IfcElementComponentType = /** @class */ (function () {
    function IfcElementComponentType(props) {
        this["@class"] = ".IfcElementComponentType";
    }
    return IfcElementComponentType;
}());
exports.IfcElementComponentType = IfcElementComponentType;
var IfcElementType = /** @class */ (function () {
    function IfcElementType(props) {
        this["@class"] = ".IfcElementType";
    }
    return IfcElementType;
}());
exports.IfcElementType = IfcElementType;
var IfcEllipse = /** @class */ (function () {
    function IfcEllipse(props) {
        this["@class"] = ".IfcEllipse";
    }
    return IfcEllipse;
}());
exports.IfcEllipse = IfcEllipse;
var IfcEllipseProfileDef = /** @class */ (function () {
    function IfcEllipseProfileDef(props) {
        this["@class"] = ".IfcEllipseProfileDef";
    }
    return IfcEllipseProfileDef;
}());
exports.IfcEllipseProfileDef = IfcEllipseProfileDef;
var IfcEnergyConversionDevice = /** @class */ (function () {
    function IfcEnergyConversionDevice(props) {
        this["@class"] = ".IfcEnergyConversionDevice";
    }
    return IfcEnergyConversionDevice;
}());
exports.IfcEnergyConversionDevice = IfcEnergyConversionDevice;
var IfcEnergyConversionDeviceType = /** @class */ (function () {
    function IfcEnergyConversionDeviceType(props) {
        this["@class"] = ".IfcEnergyConversionDeviceType";
    }
    return IfcEnergyConversionDeviceType;
}());
exports.IfcEnergyConversionDeviceType = IfcEnergyConversionDeviceType;
var IfcEngine = /** @class */ (function () {
    function IfcEngine(props) {
        this["@class"] = ".IfcEngine";
    }
    return IfcEngine;
}());
exports.IfcEngine = IfcEngine;
var IfcEngineType = /** @class */ (function () {
    function IfcEngineType(props) {
        this["@class"] = ".IfcEngineType";
    }
    return IfcEngineType;
}());
exports.IfcEngineType = IfcEngineType;
var IfcEvaporativeCooler = /** @class */ (function () {
    function IfcEvaporativeCooler(props) {
        this["@class"] = ".IfcEvaporativeCooler";
    }
    return IfcEvaporativeCooler;
}());
exports.IfcEvaporativeCooler = IfcEvaporativeCooler;
var IfcEvaporativeCoolerType = /** @class */ (function () {
    function IfcEvaporativeCoolerType(props) {
        this["@class"] = ".IfcEvaporativeCoolerType";
    }
    return IfcEvaporativeCoolerType;
}());
exports.IfcEvaporativeCoolerType = IfcEvaporativeCoolerType;
var IfcEvaporator = /** @class */ (function () {
    function IfcEvaporator(props) {
        this["@class"] = ".IfcEvaporator";
    }
    return IfcEvaporator;
}());
exports.IfcEvaporator = IfcEvaporator;
var IfcEvaporatorType = /** @class */ (function () {
    function IfcEvaporatorType(props) {
        this["@class"] = ".IfcEvaporatorType";
    }
    return IfcEvaporatorType;
}());
exports.IfcEvaporatorType = IfcEvaporatorType;
var IfcEventTime = /** @class */ (function () {
    function IfcEventTime(props) {
        this["@class"] = ".IfcEventTime";
    }
    return IfcEventTime;
}());
exports.IfcEventTime = IfcEventTime;
var IfcEventType = /** @class */ (function () {
    function IfcEventType(props) {
        this["@class"] = ".IfcEventType";
    }
    return IfcEventType;
}());
exports.IfcEventType = IfcEventType;
var IfcExternalInformation = /** @class */ (function () {
    function IfcExternalInformation(props) {
        this["@class"] = ".IfcExternalInformation";
    }
    return IfcExternalInformation;
}());
exports.IfcExternalInformation = IfcExternalInformation;
var IfcExternalReference = /** @class */ (function () {
    function IfcExternalReference(props) {
        this["@class"] = ".IfcExternalReference";
    }
    return IfcExternalReference;
}());
exports.IfcExternalReference = IfcExternalReference;
var IfcExternalSpatialElement = /** @class */ (function () {
    function IfcExternalSpatialElement(props) {
        this["@class"] = ".IfcExternalSpatialElement";
    }
    return IfcExternalSpatialElement;
}());
exports.IfcExternalSpatialElement = IfcExternalSpatialElement;
var IfcExternalSpatialStructureElement = /** @class */ (function () {
    function IfcExternalSpatialStructureElement(props) {
        this["@class"] = ".IfcExternalSpatialStructureElement";
    }
    return IfcExternalSpatialStructureElement;
}());
exports.IfcExternalSpatialStructureElement = IfcExternalSpatialStructureElement;
var IfcExternallyDefinedHatchStyle = /** @class */ (function () {
    function IfcExternallyDefinedHatchStyle(props) {
        this["@class"] = ".IfcExternallyDefinedHatchStyle";
    }
    return IfcExternallyDefinedHatchStyle;
}());
exports.IfcExternallyDefinedHatchStyle = IfcExternallyDefinedHatchStyle;
var IfcExternallyDefinedSurfaceStyle = /** @class */ (function () {
    function IfcExternallyDefinedSurfaceStyle(props) {
        this["@class"] = ".IfcExternallyDefinedSurfaceStyle";
    }
    return IfcExternallyDefinedSurfaceStyle;
}());
exports.IfcExternallyDefinedSurfaceStyle = IfcExternallyDefinedSurfaceStyle;
var IfcExternallyDefinedTextFont = /** @class */ (function () {
    function IfcExternallyDefinedTextFont(props) {
        this["@class"] = ".IfcExternallyDefinedTextFont";
    }
    return IfcExternallyDefinedTextFont;
}());
exports.IfcExternallyDefinedTextFont = IfcExternallyDefinedTextFont;
var IfcFaceOuterBound = /** @class */ (function () {
    function IfcFaceOuterBound(props) {
        this["@class"] = ".IfcFaceOuterBound";
    }
    return IfcFaceOuterBound;
}());
exports.IfcFaceOuterBound = IfcFaceOuterBound;
var IfcFacetedBrep = /** @class */ (function () {
    function IfcFacetedBrep(props) {
        this["@class"] = ".IfcFacetedBrep";
    }
    return IfcFacetedBrep;
}());
exports.IfcFacetedBrep = IfcFacetedBrep;
var IfcFailureConnectionCondition = /** @class */ (function () {
    function IfcFailureConnectionCondition(props) {
        this["@class"] = ".IfcFailureConnectionCondition";
    }
    return IfcFailureConnectionCondition;
}());
exports.IfcFailureConnectionCondition = IfcFailureConnectionCondition;
var IfcFan = /** @class */ (function () {
    function IfcFan(props) {
        this["@class"] = ".IfcFan";
    }
    return IfcFan;
}());
exports.IfcFan = IfcFan;
var IfcFanType = /** @class */ (function () {
    function IfcFanType(props) {
        this["@class"] = ".IfcFanType";
    }
    return IfcFanType;
}());
exports.IfcFanType = IfcFanType;
var IfcFastener = /** @class */ (function () {
    function IfcFastener(props) {
        this["@class"] = ".IfcFastener";
    }
    return IfcFastener;
}());
exports.IfcFastener = IfcFastener;
var IfcFastenerType = /** @class */ (function () {
    function IfcFastenerType(props) {
        this["@class"] = ".IfcFastenerType";
    }
    return IfcFastenerType;
}());
exports.IfcFastenerType = IfcFastenerType;
var IfcFeatureElement = /** @class */ (function () {
    function IfcFeatureElement(props) {
        this["@class"] = ".IfcFeatureElement";
    }
    return IfcFeatureElement;
}());
exports.IfcFeatureElement = IfcFeatureElement;
var IfcFeatureElementAddition = /** @class */ (function () {
    function IfcFeatureElementAddition(props) {
        this["@class"] = ".IfcFeatureElementAddition";
    }
    return IfcFeatureElementAddition;
}());
exports.IfcFeatureElementAddition = IfcFeatureElementAddition;
var IfcFeatureElementSubtraction = /** @class */ (function () {
    function IfcFeatureElementSubtraction(props) {
        this["@class"] = ".IfcFeatureElementSubtraction";
    }
    return IfcFeatureElementSubtraction;
}());
exports.IfcFeatureElementSubtraction = IfcFeatureElementSubtraction;
var TilingPattern = /** @class */ (function () {
    function TilingPattern(props) {
        this["@class"] = ".TilingPattern";
        if (props) {
            this.IfcVector = (props.IfcVector) ? new IfcVector(props.IfcVector) : undefined;
        }
    }
    return TilingPattern;
}());
exports.TilingPattern = TilingPattern;
var IfcFilter = /** @class */ (function () {
    function IfcFilter(props) {
        this["@class"] = ".IfcFilter";
    }
    return IfcFilter;
}());
exports.IfcFilter = IfcFilter;
var IfcFilterType = /** @class */ (function () {
    function IfcFilterType(props) {
        this["@class"] = ".IfcFilterType";
    }
    return IfcFilterType;
}());
exports.IfcFilterType = IfcFilterType;
var IfcFireSuppressionTerminal = /** @class */ (function () {
    function IfcFireSuppressionTerminal(props) {
        this["@class"] = ".IfcFireSuppressionTerminal";
    }
    return IfcFireSuppressionTerminal;
}());
exports.IfcFireSuppressionTerminal = IfcFireSuppressionTerminal;
var IfcFireSuppressionTerminalType = /** @class */ (function () {
    function IfcFireSuppressionTerminalType(props) {
        this["@class"] = ".IfcFireSuppressionTerminalType";
    }
    return IfcFireSuppressionTerminalType;
}());
exports.IfcFireSuppressionTerminalType = IfcFireSuppressionTerminalType;
var IfcFlowController = /** @class */ (function () {
    function IfcFlowController(props) {
        this["@class"] = ".IfcFlowController";
    }
    return IfcFlowController;
}());
exports.IfcFlowController = IfcFlowController;
var IfcFlowControllerType = /** @class */ (function () {
    function IfcFlowControllerType(props) {
        this["@class"] = ".IfcFlowControllerType";
    }
    return IfcFlowControllerType;
}());
exports.IfcFlowControllerType = IfcFlowControllerType;
var IfcFlowFitting = /** @class */ (function () {
    function IfcFlowFitting(props) {
        this["@class"] = ".IfcFlowFitting";
    }
    return IfcFlowFitting;
}());
exports.IfcFlowFitting = IfcFlowFitting;
var IfcFlowFittingType = /** @class */ (function () {
    function IfcFlowFittingType(props) {
        this["@class"] = ".IfcFlowFittingType";
    }
    return IfcFlowFittingType;
}());
exports.IfcFlowFittingType = IfcFlowFittingType;
var IfcFlowInstrument = /** @class */ (function () {
    function IfcFlowInstrument(props) {
        this["@class"] = ".IfcFlowInstrument";
    }
    return IfcFlowInstrument;
}());
exports.IfcFlowInstrument = IfcFlowInstrument;
var IfcFlowInstrumentType = /** @class */ (function () {
    function IfcFlowInstrumentType(props) {
        this["@class"] = ".IfcFlowInstrumentType";
    }
    return IfcFlowInstrumentType;
}());
exports.IfcFlowInstrumentType = IfcFlowInstrumentType;
var IfcFlowMeter = /** @class */ (function () {
    function IfcFlowMeter(props) {
        this["@class"] = ".IfcFlowMeter";
    }
    return IfcFlowMeter;
}());
exports.IfcFlowMeter = IfcFlowMeter;
var IfcFlowMeterType = /** @class */ (function () {
    function IfcFlowMeterType(props) {
        this["@class"] = ".IfcFlowMeterType";
    }
    return IfcFlowMeterType;
}());
exports.IfcFlowMeterType = IfcFlowMeterType;
var IfcFlowMovingDevice = /** @class */ (function () {
    function IfcFlowMovingDevice(props) {
        this["@class"] = ".IfcFlowMovingDevice";
    }
    return IfcFlowMovingDevice;
}());
exports.IfcFlowMovingDevice = IfcFlowMovingDevice;
var IfcFlowMovingDeviceType = /** @class */ (function () {
    function IfcFlowMovingDeviceType(props) {
        this["@class"] = ".IfcFlowMovingDeviceType";
    }
    return IfcFlowMovingDeviceType;
}());
exports.IfcFlowMovingDeviceType = IfcFlowMovingDeviceType;
var IfcFlowSegment = /** @class */ (function () {
    function IfcFlowSegment(props) {
        this["@class"] = ".IfcFlowSegment";
    }
    return IfcFlowSegment;
}());
exports.IfcFlowSegment = IfcFlowSegment;
var IfcFlowSegmentType = /** @class */ (function () {
    function IfcFlowSegmentType(props) {
        this["@class"] = ".IfcFlowSegmentType";
    }
    return IfcFlowSegmentType;
}());
exports.IfcFlowSegmentType = IfcFlowSegmentType;
var IfcFlowStorageDevice = /** @class */ (function () {
    function IfcFlowStorageDevice(props) {
        this["@class"] = ".IfcFlowStorageDevice";
    }
    return IfcFlowStorageDevice;
}());
exports.IfcFlowStorageDevice = IfcFlowStorageDevice;
var IfcFlowStorageDeviceType = /** @class */ (function () {
    function IfcFlowStorageDeviceType(props) {
        this["@class"] = ".IfcFlowStorageDeviceType";
    }
    return IfcFlowStorageDeviceType;
}());
exports.IfcFlowStorageDeviceType = IfcFlowStorageDeviceType;
var IfcFlowTerminal = /** @class */ (function () {
    function IfcFlowTerminal(props) {
        this["@class"] = ".IfcFlowTerminal";
    }
    return IfcFlowTerminal;
}());
exports.IfcFlowTerminal = IfcFlowTerminal;
var IfcFlowTerminalType = /** @class */ (function () {
    function IfcFlowTerminalType(props) {
        this["@class"] = ".IfcFlowTerminalType";
    }
    return IfcFlowTerminalType;
}());
exports.IfcFlowTerminalType = IfcFlowTerminalType;
var IfcFlowTreatmentDevice = /** @class */ (function () {
    function IfcFlowTreatmentDevice(props) {
        this["@class"] = ".IfcFlowTreatmentDevice";
    }
    return IfcFlowTreatmentDevice;
}());
exports.IfcFlowTreatmentDevice = IfcFlowTreatmentDevice;
var IfcFlowTreatmentDeviceType = /** @class */ (function () {
    function IfcFlowTreatmentDeviceType(props) {
        this["@class"] = ".IfcFlowTreatmentDeviceType";
    }
    return IfcFlowTreatmentDeviceType;
}());
exports.IfcFlowTreatmentDeviceType = IfcFlowTreatmentDeviceType;
var IfcFooting = /** @class */ (function () {
    function IfcFooting(props) {
        this["@class"] = ".IfcFooting";
    }
    return IfcFooting;
}());
exports.IfcFooting = IfcFooting;
var IfcFootingType = /** @class */ (function () {
    function IfcFootingType(props) {
        this["@class"] = ".IfcFootingType";
    }
    return IfcFootingType;
}());
exports.IfcFootingType = IfcFootingType;
var IfcFurnishingElement = /** @class */ (function () {
    function IfcFurnishingElement(props) {
        this["@class"] = ".IfcFurnishingElement";
    }
    return IfcFurnishingElement;
}());
exports.IfcFurnishingElement = IfcFurnishingElement;
var IfcFurnishingElementType = /** @class */ (function () {
    function IfcFurnishingElementType(props) {
        this["@class"] = ".IfcFurnishingElementType";
    }
    return IfcFurnishingElementType;
}());
exports.IfcFurnishingElementType = IfcFurnishingElementType;
var IfcFurniture = /** @class */ (function () {
    function IfcFurniture(props) {
        this["@class"] = ".IfcFurniture";
    }
    return IfcFurniture;
}());
exports.IfcFurniture = IfcFurniture;
var IfcFurnitureType = /** @class */ (function () {
    function IfcFurnitureType(props) {
        this["@class"] = ".IfcFurnitureType";
    }
    return IfcFurnitureType;
}());
exports.IfcFurnitureType = IfcFurnitureType;
var IfcGeographicElement = /** @class */ (function () {
    function IfcGeographicElement(props) {
        this["@class"] = ".IfcGeographicElement";
    }
    return IfcGeographicElement;
}());
exports.IfcGeographicElement = IfcGeographicElement;
var IfcGeographicElementType = /** @class */ (function () {
    function IfcGeographicElementType(props) {
        this["@class"] = ".IfcGeographicElementType";
    }
    return IfcGeographicElementType;
}());
exports.IfcGeographicElementType = IfcGeographicElementType;
var IfcGeometricCurveSet = /** @class */ (function () {
    function IfcGeometricCurveSet(props) {
        this["@class"] = ".IfcGeometricCurveSet";
    }
    return IfcGeometricCurveSet;
}());
exports.IfcGeometricCurveSet = IfcGeometricCurveSet;
var IfcGeometricRepresentationItem = /** @class */ (function () {
    function IfcGeometricRepresentationItem(props) {
        this["@class"] = ".IfcGeometricRepresentationItem";
    }
    return IfcGeometricRepresentationItem;
}());
exports.IfcGeometricRepresentationItem = IfcGeometricRepresentationItem;
var IfcGeometricRepresentationSubContext = /** @class */ (function () {
    function IfcGeometricRepresentationSubContext() {
    }
    return IfcGeometricRepresentationSubContext;
}());
exports.IfcGeometricRepresentationSubContext = IfcGeometricRepresentationSubContext;
-temp;
{
    constructor(props ?  : IfcGeometricRepresentationSubContext - temp);
    {
        this["@class"] = ".IfcGeometricRepresentationSubContext-temp";
    }
}
var IfcGeometricRepresentationSubContext = /** @class */ (function () {
    function IfcGeometricRepresentationSubContext(props) {
        this["@class"] = ".IfcGeometricRepresentationSubContext";
    }
    return IfcGeometricRepresentationSubContext;
}());
exports.IfcGeometricRepresentationSubContext = IfcGeometricRepresentationSubContext;
var IfcGroup = /** @class */ (function () {
    function IfcGroup(props) {
        this["@class"] = ".IfcGroup";
    }
    return IfcGroup;
}());
exports.IfcGroup = IfcGroup;
var IfcHeatExchanger = /** @class */ (function () {
    function IfcHeatExchanger(props) {
        this["@class"] = ".IfcHeatExchanger";
    }
    return IfcHeatExchanger;
}());
exports.IfcHeatExchanger = IfcHeatExchanger;
var IfcHeatExchangerType = /** @class */ (function () {
    function IfcHeatExchangerType(props) {
        this["@class"] = ".IfcHeatExchangerType";
    }
    return IfcHeatExchangerType;
}());
exports.IfcHeatExchangerType = IfcHeatExchangerType;
var IfcHumidifier = /** @class */ (function () {
    function IfcHumidifier(props) {
        this["@class"] = ".IfcHumidifier";
    }
    return IfcHumidifier;
}());
exports.IfcHumidifier = IfcHumidifier;
var IfcHumidifierType = /** @class */ (function () {
    function IfcHumidifierType(props) {
        this["@class"] = ".IfcHumidifierType";
    }
    return IfcHumidifierType;
}());
exports.IfcHumidifierType = IfcHumidifierType;
var IfcIShapeProfileDef = /** @class */ (function () {
    function IfcIShapeProfileDef(props) {
        this["@class"] = ".IfcIShapeProfileDef";
    }
    return IfcIShapeProfileDef;
}());
exports.IfcIShapeProfileDef = IfcIShapeProfileDef;
var IfcImageTexture = /** @class */ (function () {
    function IfcImageTexture(props) {
        this["@class"] = ".IfcImageTexture";
    }
    return IfcImageTexture;
}());
exports.IfcImageTexture = IfcImageTexture;
var IfcIndexedPolygonalFace = /** @class */ (function () {
    function IfcIndexedPolygonalFace(props) {
        this["@class"] = ".IfcIndexedPolygonalFace";
    }
    return IfcIndexedPolygonalFace;
}());
exports.IfcIndexedPolygonalFace = IfcIndexedPolygonalFace;
var IfcIndexedTriangleTextureMap = /** @class */ (function () {
    function IfcIndexedTriangleTextureMap(props) {
        this["@class"] = ".IfcIndexedTriangleTextureMap";
    }
    return IfcIndexedTriangleTextureMap;
}());
exports.IfcIndexedTriangleTextureMap = IfcIndexedTriangleTextureMap;
var IfcInterceptor = /** @class */ (function () {
    function IfcInterceptor(props) {
        this["@class"] = ".IfcInterceptor";
    }
    return IfcInterceptor;
}());
exports.IfcInterceptor = IfcInterceptor;
var IfcInterceptorType = /** @class */ (function () {
    function IfcInterceptorType(props) {
        this["@class"] = ".IfcInterceptorType";
    }
    return IfcInterceptorType;
}());
exports.IfcInterceptorType = IfcInterceptorType;
var IfcIntersectionCurve = /** @class */ (function () {
    function IfcIntersectionCurve(props) {
        this["@class"] = ".IfcIntersectionCurve";
    }
    return IfcIntersectionCurve;
}());
exports.IfcIntersectionCurve = IfcIntersectionCurve;
var IfcJunctionBox = /** @class */ (function () {
    function IfcJunctionBox(props) {
        this["@class"] = ".IfcJunctionBox";
    }
    return IfcJunctionBox;
}());
exports.IfcJunctionBox = IfcJunctionBox;
var IfcJunctionBoxType = /** @class */ (function () {
    function IfcJunctionBoxType(props) {
        this["@class"] = ".IfcJunctionBoxType";
    }
    return IfcJunctionBoxType;
}());
exports.IfcJunctionBoxType = IfcJunctionBoxType;
var IfcLShapeProfileDef = /** @class */ (function () {
    function IfcLShapeProfileDef(props) {
        this["@class"] = ".IfcLShapeProfileDef";
    }
    return IfcLShapeProfileDef;
}());
exports.IfcLShapeProfileDef = IfcLShapeProfileDef;
var IfcLaborResource = /** @class */ (function () {
    function IfcLaborResource(props) {
        this["@class"] = ".IfcLaborResource";
    }
    return IfcLaborResource;
}());
exports.IfcLaborResource = IfcLaborResource;
var IfcLaborResourceType = /** @class */ (function () {
    function IfcLaborResourceType(props) {
        this["@class"] = ".IfcLaborResourceType";
    }
    return IfcLaborResourceType;
}());
exports.IfcLaborResourceType = IfcLaborResourceType;
var IfcLamp = /** @class */ (function () {
    function IfcLamp(props) {
        this["@class"] = ".IfcLamp";
    }
    return IfcLamp;
}());
exports.IfcLamp = IfcLamp;
var IfcLampType = /** @class */ (function () {
    function IfcLampType(props) {
        this["@class"] = ".IfcLampType";
    }
    return IfcLampType;
}());
exports.IfcLampType = IfcLampType;
var IfcLightDistributionData = /** @class */ (function () {
    function IfcLightDistributionData(props) {
        this["@class"] = ".IfcLightDistributionData";
    }
    return IfcLightDistributionData;
}());
exports.IfcLightDistributionData = IfcLightDistributionData;
var IfcLightFixture = /** @class */ (function () {
    function IfcLightFixture(props) {
        this["@class"] = ".IfcLightFixture";
    }
    return IfcLightFixture;
}());
exports.IfcLightFixture = IfcLightFixture;
var IfcLightFixtureType = /** @class */ (function () {
    function IfcLightFixtureType(props) {
        this["@class"] = ".IfcLightFixtureType";
    }
    return IfcLightFixtureType;
}());
exports.IfcLightFixtureType = IfcLightFixtureType;
var IfcLightSourceAmbient = /** @class */ (function () {
    function IfcLightSourceAmbient(props) {
        this["@class"] = ".IfcLightSourceAmbient";
    }
    return IfcLightSourceAmbient;
}());
exports.IfcLightSourceAmbient = IfcLightSourceAmbient;
var IfcLoop = /** @class */ (function () {
    function IfcLoop(props) {
        this["@class"] = ".IfcLoop";
    }
    return IfcLoop;
}());
exports.IfcLoop = IfcLoop;
var IfcMapConversion = /** @class */ (function () {
    function IfcMapConversion(props) {
        this["@class"] = ".IfcMapConversion";
    }
    return IfcMapConversion;
}());
exports.IfcMapConversion = IfcMapConversion;
var IfcMaterialDefinitionRepresentation = /** @class */ (function () {
    function IfcMaterialDefinitionRepresentation(props) {
        this["@class"] = ".IfcMaterialDefinitionRepresentation";
    }
    return IfcMaterialDefinitionRepresentation;
}());
exports.IfcMaterialDefinitionRepresentation = IfcMaterialDefinitionRepresentation;
var IfcMaterialLayerWithOffsets = /** @class */ (function () {
    function IfcMaterialLayerWithOffsets(props) {
        this["@class"] = ".IfcMaterialLayerWithOffsets";
    }
    return IfcMaterialLayerWithOffsets;
}());
exports.IfcMaterialLayerWithOffsets = IfcMaterialLayerWithOffsets;
var IfcMaterialProfileWithOffsets = /** @class */ (function () {
    function IfcMaterialProfileWithOffsets(props) {
        this["@class"] = ".IfcMaterialProfileWithOffsets";
    }
    return IfcMaterialProfileWithOffsets;
}());
exports.IfcMaterialProfileWithOffsets = IfcMaterialProfileWithOffsets;
var IfcMaterialProperties = /** @class */ (function () {
    function IfcMaterialProperties(props) {
        this["@class"] = ".IfcMaterialProperties";
    }
    return IfcMaterialProperties;
}());
exports.IfcMaterialProperties = IfcMaterialProperties;
var IfcMaterialUsageDefinition = /** @class */ (function () {
    function IfcMaterialUsageDefinition(props) {
        this["@class"] = ".IfcMaterialUsageDefinition";
    }
    return IfcMaterialUsageDefinition;
}());
exports.IfcMaterialUsageDefinition = IfcMaterialUsageDefinition;
var IfcMechanicalFastener = /** @class */ (function () {
    function IfcMechanicalFastener(props) {
        this["@class"] = ".IfcMechanicalFastener";
    }
    return IfcMechanicalFastener;
}());
exports.IfcMechanicalFastener = IfcMechanicalFastener;
var IfcMechanicalFastenerType = /** @class */ (function () {
    function IfcMechanicalFastenerType(props) {
        this["@class"] = ".IfcMechanicalFastenerType";
    }
    return IfcMechanicalFastenerType;
}());
exports.IfcMechanicalFastenerType = IfcMechanicalFastenerType;
var IfcMedicalDevice = /** @class */ (function () {
    function IfcMedicalDevice(props) {
        this["@class"] = ".IfcMedicalDevice";
    }
    return IfcMedicalDevice;
}());
exports.IfcMedicalDevice = IfcMedicalDevice;
var IfcMedicalDeviceType = /** @class */ (function () {
    function IfcMedicalDeviceType(props) {
        this["@class"] = ".IfcMedicalDeviceType";
    }
    return IfcMedicalDeviceType;
}());
exports.IfcMedicalDeviceType = IfcMedicalDeviceType;
var IfcMember = /** @class */ (function () {
    function IfcMember(props) {
        this["@class"] = ".IfcMember";
    }
    return IfcMember;
}());
exports.IfcMember = IfcMember;
var IfcMemberStandardCase = /** @class */ (function () {
    function IfcMemberStandardCase(props) {
        this["@class"] = ".IfcMemberStandardCase";
    }
    return IfcMemberStandardCase;
}());
exports.IfcMemberStandardCase = IfcMemberStandardCase;
var IfcMemberType = /** @class */ (function () {
    function IfcMemberType(props) {
        this["@class"] = ".IfcMemberType";
    }
    return IfcMemberType;
}());
exports.IfcMemberType = IfcMemberType;
var IfcMirroredProfileDef = /** @class */ (function () {
    function IfcMirroredProfileDef() {
    }
    return IfcMirroredProfileDef;
}());
exports.IfcMirroredProfileDef = IfcMirroredProfileDef;
-temp;
{
    constructor(props ?  : IfcMirroredProfileDef - temp);
    {
        this["@class"] = ".IfcMirroredProfileDef-temp";
    }
}
var IfcMirroredProfileDef = /** @class */ (function () {
    function IfcMirroredProfileDef(props) {
        this["@class"] = ".IfcMirroredProfileDef";
    }
    return IfcMirroredProfileDef;
}());
exports.IfcMirroredProfileDef = IfcMirroredProfileDef;
var IfcMonetaryUnit = /** @class */ (function () {
    function IfcMonetaryUnit(props) {
        this["@class"] = ".IfcMonetaryUnit";
    }
    return IfcMonetaryUnit;
}());
exports.IfcMonetaryUnit = IfcMonetaryUnit;
var IfcMotorConnection = /** @class */ (function () {
    function IfcMotorConnection(props) {
        this["@class"] = ".IfcMotorConnection";
    }
    return IfcMotorConnection;
}());
exports.IfcMotorConnection = IfcMotorConnection;
var IfcMotorConnectionType = /** @class */ (function () {
    function IfcMotorConnectionType(props) {
        this["@class"] = ".IfcMotorConnectionType";
    }
    return IfcMotorConnectionType;
}());
exports.IfcMotorConnectionType = IfcMotorConnectionType;
var IfcObjectPlacement = /** @class */ (function () {
    function IfcObjectPlacement(props) {
        this["@class"] = ".IfcObjectPlacement";
    }
    return IfcObjectPlacement;
}());
exports.IfcObjectPlacement = IfcObjectPlacement;
var IfcOccupant = /** @class */ (function () {
    function IfcOccupant(props) {
        this["@class"] = ".IfcOccupant";
    }
    return IfcOccupant;
}());
exports.IfcOccupant = IfcOccupant;
var IfcOpenShell = /** @class */ (function () {
    function IfcOpenShell(props) {
        this["@class"] = ".IfcOpenShell";
    }
    return IfcOpenShell;
}());
exports.IfcOpenShell = IfcOpenShell;
var IfcOpeningStandardCase = /** @class */ (function () {
    function IfcOpeningStandardCase(props) {
        this["@class"] = ".IfcOpeningStandardCase";
    }
    return IfcOpeningStandardCase;
}());
exports.IfcOpeningStandardCase = IfcOpeningStandardCase;
var IfcOrientedEdge = /** @class */ (function () {
    function IfcOrientedEdge() {
    }
    return IfcOrientedEdge;
}());
exports.IfcOrientedEdge = IfcOrientedEdge;
-temp;
{
    constructor(props ?  : IfcOrientedEdge - temp);
    {
        this["@class"] = ".IfcOrientedEdge-temp";
    }
}
var IfcOuterBoundaryCurve = /** @class */ (function () {
    function IfcOuterBoundaryCurve(props) {
        this["@class"] = ".IfcOuterBoundaryCurve";
    }
    return IfcOuterBoundaryCurve;
}());
exports.IfcOuterBoundaryCurve = IfcOuterBoundaryCurve;
var IfcOutlet = /** @class */ (function () {
    function IfcOutlet(props) {
        this["@class"] = ".IfcOutlet";
    }
    return IfcOutlet;
}());
exports.IfcOutlet = IfcOutlet;
var IfcOutletType = /** @class */ (function () {
    function IfcOutletType(props) {
        this["@class"] = ".IfcOutletType";
    }
    return IfcOutletType;
}());
exports.IfcOutletType = IfcOutletType;
var IfcPerformanceHistory = /** @class */ (function () {
    function IfcPerformanceHistory(props) {
        this["@class"] = ".IfcPerformanceHistory";
    }
    return IfcPerformanceHistory;
}());
exports.IfcPerformanceHistory = IfcPerformanceHistory;
var IfcPermit = /** @class */ (function () {
    function IfcPermit(props) {
        this["@class"] = ".IfcPermit";
    }
    return IfcPermit;
}());
exports.IfcPermit = IfcPermit;
var IfcPhysicalQuantity = /** @class */ (function () {
    function IfcPhysicalQuantity(props) {
        this["@class"] = ".IfcPhysicalQuantity";
    }
    return IfcPhysicalQuantity;
}());
exports.IfcPhysicalQuantity = IfcPhysicalQuantity;
var IfcPile = /** @class */ (function () {
    function IfcPile(props) {
        this["@class"] = ".IfcPile";
    }
    return IfcPile;
}());
exports.IfcPile = IfcPile;
var IfcPileType = /** @class */ (function () {
    function IfcPileType(props) {
        this["@class"] = ".IfcPileType";
    }
    return IfcPileType;
}());
exports.IfcPileType = IfcPileType;
var IfcPipeFitting = /** @class */ (function () {
    function IfcPipeFitting(props) {
        this["@class"] = ".IfcPipeFitting";
    }
    return IfcPipeFitting;
}());
exports.IfcPipeFitting = IfcPipeFitting;
var IfcPipeFittingType = /** @class */ (function () {
    function IfcPipeFittingType(props) {
        this["@class"] = ".IfcPipeFittingType";
    }
    return IfcPipeFittingType;
}());
exports.IfcPipeFittingType = IfcPipeFittingType;
var IfcPipeSegment = /** @class */ (function () {
    function IfcPipeSegment(props) {
        this["@class"] = ".IfcPipeSegment";
    }
    return IfcPipeSegment;
}());
exports.IfcPipeSegment = IfcPipeSegment;
var IfcPipeSegmentType = /** @class */ (function () {
    function IfcPipeSegmentType(props) {
        this["@class"] = ".IfcPipeSegmentType";
    }
    return IfcPipeSegmentType;
}());
exports.IfcPipeSegmentType = IfcPipeSegmentType;
var IfcPixelTexture = /** @class */ (function () {
    function IfcPixelTexture(props) {
        this["@class"] = ".IfcPixelTexture";
    }
    return IfcPixelTexture;
}());
exports.IfcPixelTexture = IfcPixelTexture;
var IfcPlanarExtent = /** @class */ (function () {
    function IfcPlanarExtent(props) {
        this["@class"] = ".IfcPlanarExtent";
    }
    return IfcPlanarExtent;
}());
exports.IfcPlanarExtent = IfcPlanarExtent;
var IfcPlane = /** @class */ (function () {
    function IfcPlane(props) {
        this["@class"] = ".IfcPlane";
    }
    return IfcPlane;
}());
exports.IfcPlane = IfcPlane;
var IfcPlate = /** @class */ (function () {
    function IfcPlate(props) {
        this["@class"] = ".IfcPlate";
    }
    return IfcPlate;
}());
exports.IfcPlate = IfcPlate;
var IfcPlateStandardCase = /** @class */ (function () {
    function IfcPlateStandardCase(props) {
        this["@class"] = ".IfcPlateStandardCase";
    }
    return IfcPlateStandardCase;
}());
exports.IfcPlateStandardCase = IfcPlateStandardCase;
var IfcPlateType = /** @class */ (function () {
    function IfcPlateType(props) {
        this["@class"] = ".IfcPlateType";
    }
    return IfcPlateType;
}());
exports.IfcPlateType = IfcPlateType;
var IfcPoint = /** @class */ (function () {
    function IfcPoint(props) {
        this["@class"] = ".IfcPoint";
    }
    return IfcPoint;
}());
exports.IfcPoint = IfcPoint;
var IfcPort = /** @class */ (function () {
    function IfcPort(props) {
        this["@class"] = ".IfcPort";
    }
    return IfcPort;
}());
exports.IfcPort = IfcPort;
var IfcPreDefinedColour = /** @class */ (function () {
    function IfcPreDefinedColour(props) {
        this["@class"] = ".IfcPreDefinedColour";
    }
    return IfcPreDefinedColour;
}());
exports.IfcPreDefinedColour = IfcPreDefinedColour;
var IfcPreDefinedCurveFont = /** @class */ (function () {
    function IfcPreDefinedCurveFont(props) {
        this["@class"] = ".IfcPreDefinedCurveFont";
    }
    return IfcPreDefinedCurveFont;
}());
exports.IfcPreDefinedCurveFont = IfcPreDefinedCurveFont;
var IfcPreDefinedItem = /** @class */ (function () {
    function IfcPreDefinedItem(props) {
        this["@class"] = ".IfcPreDefinedItem";
    }
    return IfcPreDefinedItem;
}());
exports.IfcPreDefinedItem = IfcPreDefinedItem;
var IfcPreDefinedProperties = /** @class */ (function () {
    function IfcPreDefinedProperties(props) {
        this["@class"] = ".IfcPreDefinedProperties";
    }
    return IfcPreDefinedProperties;
}());
exports.IfcPreDefinedProperties = IfcPreDefinedProperties;
var IfcPreDefinedPropertySet = /** @class */ (function () {
    function IfcPreDefinedPropertySet(props) {
        this["@class"] = ".IfcPreDefinedPropertySet";
    }
    return IfcPreDefinedPropertySet;
}());
exports.IfcPreDefinedPropertySet = IfcPreDefinedPropertySet;
var IfcPreDefinedTextFont = /** @class */ (function () {
    function IfcPreDefinedTextFont(props) {
        this["@class"] = ".IfcPreDefinedTextFont";
    }
    return IfcPreDefinedTextFont;
}());
exports.IfcPreDefinedTextFont = IfcPreDefinedTextFont;
var IfcPresentationItem = /** @class */ (function () {
    function IfcPresentationItem(props) {
        this["@class"] = ".IfcPresentationItem";
    }
    return IfcPresentationItem;
}());
exports.IfcPresentationItem = IfcPresentationItem;
var IfcPresentationStyle = /** @class */ (function () {
    function IfcPresentationStyle(props) {
        this["@class"] = ".IfcPresentationStyle";
    }
    return IfcPresentationStyle;
}());
exports.IfcPresentationStyle = IfcPresentationStyle;
var IfcProcedure = /** @class */ (function () {
    function IfcProcedure(props) {
        this["@class"] = ".IfcProcedure";
    }
    return IfcProcedure;
}());
exports.IfcProcedure = IfcProcedure;
var IfcProcedureType = /** @class */ (function () {
    function IfcProcedureType(props) {
        this["@class"] = ".IfcProcedureType";
    }
    return IfcProcedureType;
}());
exports.IfcProcedureType = IfcProcedureType;
var IfcProcess = /** @class */ (function () {
    function IfcProcess(props) {
        this["@class"] = ".IfcProcess";
    }
    return IfcProcess;
}());
exports.IfcProcess = IfcProcess;
var IfcProfileProperties = /** @class */ (function () {
    function IfcProfileProperties(props) {
        this["@class"] = ".IfcProfileProperties";
    }
    return IfcProfileProperties;
}());
exports.IfcProfileProperties = IfcProfileProperties;
var IfcProject = /** @class */ (function () {
    function IfcProject(props) {
        this["@class"] = ".IfcProject";
    }
    return IfcProject;
}());
exports.IfcProject = IfcProject;
var IfcProjectLibrary = /** @class */ (function () {
    function IfcProjectLibrary(props) {
        this["@class"] = ".IfcProjectLibrary";
    }
    return IfcProjectLibrary;
}());
exports.IfcProjectLibrary = IfcProjectLibrary;
var IfcProjectOrder = /** @class */ (function () {
    function IfcProjectOrder(props) {
        this["@class"] = ".IfcProjectOrder";
    }
    return IfcProjectOrder;
}());
exports.IfcProjectOrder = IfcProjectOrder;
var IfcProjectionElement = /** @class */ (function () {
    function IfcProjectionElement(props) {
        this["@class"] = ".IfcProjectionElement";
    }
    return IfcProjectionElement;
}());
exports.IfcProjectionElement = IfcProjectionElement;
var IfcProperty = /** @class */ (function () {
    function IfcProperty(props) {
        this["@class"] = ".IfcProperty";
    }
    return IfcProperty;
}());
exports.IfcProperty = IfcProperty;
var IfcPropertyAbstraction = /** @class */ (function () {
    function IfcPropertyAbstraction(props) {
        this["@class"] = ".IfcPropertyAbstraction";
    }
    return IfcPropertyAbstraction;
}());
exports.IfcPropertyAbstraction = IfcPropertyAbstraction;
var IfcPropertyDefinition = /** @class */ (function () {
    function IfcPropertyDefinition(props) {
        this["@class"] = ".IfcPropertyDefinition";
    }
    return IfcPropertyDefinition;
}());
exports.IfcPropertyDefinition = IfcPropertyDefinition;
var IfcPropertySetDefinition = /** @class */ (function () {
    function IfcPropertySetDefinition(props) {
        this["@class"] = ".IfcPropertySetDefinition";
    }
    return IfcPropertySetDefinition;
}());
exports.IfcPropertySetDefinition = IfcPropertySetDefinition;
var IfcPropertyTemplate = /** @class */ (function () {
    function IfcPropertyTemplate(props) {
        this["@class"] = ".IfcPropertyTemplate";
    }
    return IfcPropertyTemplate;
}());
exports.IfcPropertyTemplate = IfcPropertyTemplate;
var IfcPropertyTemplateDefinition = /** @class */ (function () {
    function IfcPropertyTemplateDefinition(props) {
        this["@class"] = ".IfcPropertyTemplateDefinition";
    }
    return IfcPropertyTemplateDefinition;
}());
exports.IfcPropertyTemplateDefinition = IfcPropertyTemplateDefinition;
var IfcProtectiveDevice = /** @class */ (function () {
    function IfcProtectiveDevice(props) {
        this["@class"] = ".IfcProtectiveDevice";
    }
    return IfcProtectiveDevice;
}());
exports.IfcProtectiveDevice = IfcProtectiveDevice;
var IfcProtectiveDeviceTrippingUnit = /** @class */ (function () {
    function IfcProtectiveDeviceTrippingUnit(props) {
        this["@class"] = ".IfcProtectiveDeviceTrippingUnit";
    }
    return IfcProtectiveDeviceTrippingUnit;
}());
exports.IfcProtectiveDeviceTrippingUnit = IfcProtectiveDeviceTrippingUnit;
var IfcProtectiveDeviceTrippingUnitType = /** @class */ (function () {
    function IfcProtectiveDeviceTrippingUnitType(props) {
        this["@class"] = ".IfcProtectiveDeviceTrippingUnitType";
    }
    return IfcProtectiveDeviceTrippingUnitType;
}());
exports.IfcProtectiveDeviceTrippingUnitType = IfcProtectiveDeviceTrippingUnitType;
var IfcProtectiveDeviceType = /** @class */ (function () {
    function IfcProtectiveDeviceType(props) {
        this["@class"] = ".IfcProtectiveDeviceType";
    }
    return IfcProtectiveDeviceType;
}());
exports.IfcProtectiveDeviceType = IfcProtectiveDeviceType;
var IfcProxy = /** @class */ (function () {
    function IfcProxy(props) {
        this["@class"] = ".IfcProxy";
    }
    return IfcProxy;
}());
exports.IfcProxy = IfcProxy;
var IfcPump = /** @class */ (function () {
    function IfcPump(props) {
        this["@class"] = ".IfcPump";
    }
    return IfcPump;
}());
exports.IfcPump = IfcPump;
var IfcPumpType = /** @class */ (function () {
    function IfcPumpType(props) {
        this["@class"] = ".IfcPumpType";
    }
    return IfcPumpType;
}());
exports.IfcPumpType = IfcPumpType;
var IfcQuantityArea = /** @class */ (function () {
    function IfcQuantityArea(props) {
        this["@class"] = ".IfcQuantityArea";
    }
    return IfcQuantityArea;
}());
exports.IfcQuantityArea = IfcQuantityArea;
var IfcQuantityCount = /** @class */ (function () {
    function IfcQuantityCount(props) {
        this["@class"] = ".IfcQuantityCount";
    }
    return IfcQuantityCount;
}());
exports.IfcQuantityCount = IfcQuantityCount;
var IfcQuantityLength = /** @class */ (function () {
    function IfcQuantityLength(props) {
        this["@class"] = ".IfcQuantityLength";
    }
    return IfcQuantityLength;
}());
exports.IfcQuantityLength = IfcQuantityLength;
var IfcQuantitySet = /** @class */ (function () {
    function IfcQuantitySet(props) {
        this["@class"] = ".IfcQuantitySet";
    }
    return IfcQuantitySet;
}());
exports.IfcQuantitySet = IfcQuantitySet;
var IfcQuantityTime = /** @class */ (function () {
    function IfcQuantityTime(props) {
        this["@class"] = ".IfcQuantityTime";
    }
    return IfcQuantityTime;
}());
exports.IfcQuantityTime = IfcQuantityTime;
var IfcQuantityVolume = /** @class */ (function () {
    function IfcQuantityVolume(props) {
        this["@class"] = ".IfcQuantityVolume";
    }
    return IfcQuantityVolume;
}());
exports.IfcQuantityVolume = IfcQuantityVolume;
var IfcQuantityWeight = /** @class */ (function () {
    function IfcQuantityWeight(props) {
        this["@class"] = ".IfcQuantityWeight";
    }
    return IfcQuantityWeight;
}());
exports.IfcQuantityWeight = IfcQuantityWeight;
var IfcRailing = /** @class */ (function () {
    function IfcRailing(props) {
        this["@class"] = ".IfcRailing";
    }
    return IfcRailing;
}());
exports.IfcRailing = IfcRailing;
var IfcRailingType = /** @class */ (function () {
    function IfcRailingType(props) {
        this["@class"] = ".IfcRailingType";
    }
    return IfcRailingType;
}());
exports.IfcRailingType = IfcRailingType;
var IfcRamp = /** @class */ (function () {
    function IfcRamp(props) {
        this["@class"] = ".IfcRamp";
    }
    return IfcRamp;
}());
exports.IfcRamp = IfcRamp;
var IfcRampFlight = /** @class */ (function () {
    function IfcRampFlight(props) {
        this["@class"] = ".IfcRampFlight";
    }
    return IfcRampFlight;
}());
exports.IfcRampFlight = IfcRampFlight;
var IfcRampFlightType = /** @class */ (function () {
    function IfcRampFlightType(props) {
        this["@class"] = ".IfcRampFlightType";
    }
    return IfcRampFlightType;
}());
exports.IfcRampFlightType = IfcRampFlightType;
var IfcRampType = /** @class */ (function () {
    function IfcRampType(props) {
        this["@class"] = ".IfcRampType";
    }
    return IfcRampType;
}());
exports.IfcRampType = IfcRampType;
var IfcRationalBSplineCurveWithKnots = /** @class */ (function () {
    function IfcRationalBSplineCurveWithKnots(props) {
        this["@class"] = ".IfcRationalBSplineCurveWithKnots";
    }
    return IfcRationalBSplineCurveWithKnots;
}());
exports.IfcRationalBSplineCurveWithKnots = IfcRationalBSplineCurveWithKnots;
var IfcRectangleHollowProfileDef = /** @class */ (function () {
    function IfcRectangleHollowProfileDef(props) {
        this["@class"] = ".IfcRectangleHollowProfileDef";
    }
    return IfcRectangleHollowProfileDef;
}());
exports.IfcRectangleHollowProfileDef = IfcRectangleHollowProfileDef;
var IfcRectangleProfileDef = /** @class */ (function () {
    function IfcRectangleProfileDef(props) {
        this["@class"] = ".IfcRectangleProfileDef";
    }
    return IfcRectangleProfileDef;
}());
exports.IfcRectangleProfileDef = IfcRectangleProfileDef;
var IfcRectangularPyramid = /** @class */ (function () {
    function IfcRectangularPyramid(props) {
        this["@class"] = ".IfcRectangularPyramid";
    }
    return IfcRectangularPyramid;
}());
exports.IfcRectangularPyramid = IfcRectangularPyramid;
var IfcReinforcementBarProperties = /** @class */ (function () {
    function IfcReinforcementBarProperties(props) {
        this["@class"] = ".IfcReinforcementBarProperties";
    }
    return IfcReinforcementBarProperties;
}());
exports.IfcReinforcementBarProperties = IfcReinforcementBarProperties;
var IfcReinforcingBar = /** @class */ (function () {
    function IfcReinforcingBar(props) {
        this["@class"] = ".IfcReinforcingBar";
    }
    return IfcReinforcingBar;
}());
exports.IfcReinforcingBar = IfcReinforcingBar;
var IfcReinforcingElement = /** @class */ (function () {
    function IfcReinforcingElement(props) {
        this["@class"] = ".IfcReinforcingElement";
    }
    return IfcReinforcingElement;
}());
exports.IfcReinforcingElement = IfcReinforcingElement;
var IfcReinforcingElementType = /** @class */ (function () {
    function IfcReinforcingElementType(props) {
        this["@class"] = ".IfcReinforcingElementType";
    }
    return IfcReinforcingElementType;
}());
exports.IfcReinforcingElementType = IfcReinforcingElementType;
var IfcReinforcingMesh = /** @class */ (function () {
    function IfcReinforcingMesh(props) {
        this["@class"] = ".IfcReinforcingMesh";
    }
    return IfcReinforcingMesh;
}());
exports.IfcReinforcingMesh = IfcReinforcingMesh;
var IfcRelAssignsToGroupByFactor = /** @class */ (function () {
    function IfcRelAssignsToGroupByFactor(props) {
        this["@class"] = ".IfcRelAssignsToGroupByFactor";
    }
    return IfcRelAssignsToGroupByFactor;
}());
exports.IfcRelAssignsToGroupByFactor = IfcRelAssignsToGroupByFactor;
var IfcRelConnects = /** @class */ (function () {
    function IfcRelConnects(props) {
        this["@class"] = ".IfcRelConnects";
    }
    return IfcRelConnects;
}());
exports.IfcRelConnects = IfcRelConnects;
var IfcRelConnectsPathElements = /** @class */ (function () {
    function IfcRelConnectsPathElements(props) {
        this["@class"] = ".IfcRelConnectsPathElements";
    }
    return IfcRelConnectsPathElements;
}());
exports.IfcRelConnectsPathElements = IfcRelConnectsPathElements;
var IfcRelDecomposes = /** @class */ (function () {
    function IfcRelDecomposes(props) {
        this["@class"] = ".IfcRelDecomposes";
    }
    return IfcRelDecomposes;
}());
exports.IfcRelDecomposes = IfcRelDecomposes;
var IfcRelDefines = /** @class */ (function () {
    function IfcRelDefines(props) {
        this["@class"] = ".IfcRelDefines";
    }
    return IfcRelDefines;
}());
exports.IfcRelDefines = IfcRelDefines;
var IfcRelationship = /** @class */ (function () {
    function IfcRelationship(props) {
        this["@class"] = ".IfcRelationship";
    }
    return IfcRelationship;
}());
exports.IfcRelationship = IfcRelationship;
var IfcReparametrisedCompositeCurveSegment = /** @class */ (function () {
    function IfcReparametrisedCompositeCurveSegment(props) {
        this["@class"] = ".IfcReparametrisedCompositeCurveSegment";
    }
    return IfcReparametrisedCompositeCurveSegment;
}());
exports.IfcReparametrisedCompositeCurveSegment = IfcReparametrisedCompositeCurveSegment;
var IfcRepresentationContext = /** @class */ (function () {
    function IfcRepresentationContext(props) {
        this["@class"] = ".IfcRepresentationContext";
    }
    return IfcRepresentationContext;
}());
exports.IfcRepresentationContext = IfcRepresentationContext;
var IfcResource = /** @class */ (function () {
    function IfcResource(props) {
        this["@class"] = ".IfcResource";
    }
    return IfcResource;
}());
exports.IfcResource = IfcResource;
var IfcResourceLevelRelationship = /** @class */ (function () {
    function IfcResourceLevelRelationship(props) {
        this["@class"] = ".IfcResourceLevelRelationship";
    }
    return IfcResourceLevelRelationship;
}());
exports.IfcResourceLevelRelationship = IfcResourceLevelRelationship;
var IfcResourceTime = /** @class */ (function () {
    function IfcResourceTime(props) {
        this["@class"] = ".IfcResourceTime";
    }
    return IfcResourceTime;
}());
exports.IfcResourceTime = IfcResourceTime;
var IfcRightCircularCone = /** @class */ (function () {
    function IfcRightCircularCone(props) {
        this["@class"] = ".IfcRightCircularCone";
    }
    return IfcRightCircularCone;
}());
exports.IfcRightCircularCone = IfcRightCircularCone;
var IfcRightCircularCylinder = /** @class */ (function () {
    function IfcRightCircularCylinder(props) {
        this["@class"] = ".IfcRightCircularCylinder";
    }
    return IfcRightCircularCylinder;
}());
exports.IfcRightCircularCylinder = IfcRightCircularCylinder;
var IfcRoof = /** @class */ (function () {
    function IfcRoof(props) {
        this["@class"] = ".IfcRoof";
    }
    return IfcRoof;
}());
exports.IfcRoof = IfcRoof;
var IfcRoofType = /** @class */ (function () {
    function IfcRoofType(props) {
        this["@class"] = ".IfcRoofType";
    }
    return IfcRoofType;
}());
exports.IfcRoofType = IfcRoofType;
var IfcRoundedRectangleProfileDef = /** @class */ (function () {
    function IfcRoundedRectangleProfileDef(props) {
        this["@class"] = ".IfcRoundedRectangleProfileDef";
    }
    return IfcRoundedRectangleProfileDef;
}());
exports.IfcRoundedRectangleProfileDef = IfcRoundedRectangleProfileDef;
var IfcSIUnit = /** @class */ (function () {
    function IfcSIUnit() {
    }
    return IfcSIUnit;
}());
exports.IfcSIUnit = IfcSIUnit;
-temp;
{
    constructor(props ?  : IfcSIUnit - temp);
    {
        this["@class"] = ".IfcSIUnit-temp";
    }
}
var IfcSIUnit = /** @class */ (function () {
    function IfcSIUnit(props) {
        this["@class"] = ".IfcSIUnit";
    }
    return IfcSIUnit;
}());
exports.IfcSIUnit = IfcSIUnit;
var IfcSanitaryTerminal = /** @class */ (function () {
    function IfcSanitaryTerminal(props) {
        this["@class"] = ".IfcSanitaryTerminal";
    }
    return IfcSanitaryTerminal;
}());
exports.IfcSanitaryTerminal = IfcSanitaryTerminal;
var IfcSanitaryTerminalType = /** @class */ (function () {
    function IfcSanitaryTerminalType(props) {
        this["@class"] = ".IfcSanitaryTerminalType";
    }
    return IfcSanitaryTerminalType;
}());
exports.IfcSanitaryTerminalType = IfcSanitaryTerminalType;
var IfcSchedulingTime = /** @class */ (function () {
    function IfcSchedulingTime(props) {
        this["@class"] = ".IfcSchedulingTime";
    }
    return IfcSchedulingTime;
}());
exports.IfcSchedulingTime = IfcSchedulingTime;
var IfcSeamCurve = /** @class */ (function () {
    function IfcSeamCurve(props) {
        this["@class"] = ".IfcSeamCurve";
    }
    return IfcSeamCurve;
}());
exports.IfcSeamCurve = IfcSeamCurve;
var IfcSensor = /** @class */ (function () {
    function IfcSensor(props) {
        this["@class"] = ".IfcSensor";
    }
    return IfcSensor;
}());
exports.IfcSensor = IfcSensor;
var IfcSensorType = /** @class */ (function () {
    function IfcSensorType(props) {
        this["@class"] = ".IfcSensorType";
    }
    return IfcSensorType;
}());
exports.IfcSensorType = IfcSensorType;
var IfcShadingDevice = /** @class */ (function () {
    function IfcShadingDevice(props) {
        this["@class"] = ".IfcShadingDevice";
    }
    return IfcShadingDevice;
}());
exports.IfcShadingDevice = IfcShadingDevice;
var IfcShadingDeviceType = /** @class */ (function () {
    function IfcShadingDeviceType(props) {
        this["@class"] = ".IfcShadingDeviceType";
    }
    return IfcShadingDeviceType;
}());
exports.IfcShadingDeviceType = IfcShadingDeviceType;
var IfcShapeModel = /** @class */ (function () {
    function IfcShapeModel(props) {
        this["@class"] = ".IfcShapeModel";
    }
    return IfcShapeModel;
}());
exports.IfcShapeModel = IfcShapeModel;
var IfcShapeRepresentation = /** @class */ (function () {
    function IfcShapeRepresentation(props) {
        this["@class"] = ".IfcShapeRepresentation";
    }
    return IfcShapeRepresentation;
}());
exports.IfcShapeRepresentation = IfcShapeRepresentation;
var IfcSimpleProperty = /** @class */ (function () {
    function IfcSimpleProperty(props) {
        this["@class"] = ".IfcSimpleProperty";
    }
    return IfcSimpleProperty;
}());
exports.IfcSimpleProperty = IfcSimpleProperty;
var IfcSlab = /** @class */ (function () {
    function IfcSlab(props) {
        this["@class"] = ".IfcSlab";
    }
    return IfcSlab;
}());
exports.IfcSlab = IfcSlab;
var IfcSlabElementedCase = /** @class */ (function () {
    function IfcSlabElementedCase(props) {
        this["@class"] = ".IfcSlabElementedCase";
    }
    return IfcSlabElementedCase;
}());
exports.IfcSlabElementedCase = IfcSlabElementedCase;
var IfcSlabStandardCase = /** @class */ (function () {
    function IfcSlabStandardCase(props) {
        this["@class"] = ".IfcSlabStandardCase";
    }
    return IfcSlabStandardCase;
}());
exports.IfcSlabStandardCase = IfcSlabStandardCase;
var IfcSlabType = /** @class */ (function () {
    function IfcSlabType(props) {
        this["@class"] = ".IfcSlabType";
    }
    return IfcSlabType;
}());
exports.IfcSlabType = IfcSlabType;
var IfcSlippageConnectionCondition = /** @class */ (function () {
    function IfcSlippageConnectionCondition(props) {
        this["@class"] = ".IfcSlippageConnectionCondition";
    }
    return IfcSlippageConnectionCondition;
}());
exports.IfcSlippageConnectionCondition = IfcSlippageConnectionCondition;
var IfcSolarDevice = /** @class */ (function () {
    function IfcSolarDevice(props) {
        this["@class"] = ".IfcSolarDevice";
    }
    return IfcSolarDevice;
}());
exports.IfcSolarDevice = IfcSolarDevice;
var IfcSolarDeviceType = /** @class */ (function () {
    function IfcSolarDeviceType(props) {
        this["@class"] = ".IfcSolarDeviceType";
    }
    return IfcSolarDeviceType;
}());
exports.IfcSolarDeviceType = IfcSolarDeviceType;
var IfcSolidModel = /** @class */ (function () {
    function IfcSolidModel(props) {
        this["@class"] = ".IfcSolidModel";
    }
    return IfcSolidModel;
}());
exports.IfcSolidModel = IfcSolidModel;
var IfcSpace = /** @class */ (function () {
    function IfcSpace(props) {
        this["@class"] = ".IfcSpace";
    }
    return IfcSpace;
}());
exports.IfcSpace = IfcSpace;
var IfcSpaceHeater = /** @class */ (function () {
    function IfcSpaceHeater(props) {
        this["@class"] = ".IfcSpaceHeater";
    }
    return IfcSpaceHeater;
}());
exports.IfcSpaceHeater = IfcSpaceHeater;
var IfcSpaceHeaterType = /** @class */ (function () {
    function IfcSpaceHeaterType(props) {
        this["@class"] = ".IfcSpaceHeaterType";
    }
    return IfcSpaceHeaterType;
}());
exports.IfcSpaceHeaterType = IfcSpaceHeaterType;
var IfcSpaceType = /** @class */ (function () {
    function IfcSpaceType(props) {
        this["@class"] = ".IfcSpaceType";
    }
    return IfcSpaceType;
}());
exports.IfcSpaceType = IfcSpaceType;
var IfcSpatialElementType = /** @class */ (function () {
    function IfcSpatialElementType(props) {
        this["@class"] = ".IfcSpatialElementType";
    }
    return IfcSpatialElementType;
}());
exports.IfcSpatialElementType = IfcSpatialElementType;
var IfcSpatialStructureElement = /** @class */ (function () {
    function IfcSpatialStructureElement(props) {
        this["@class"] = ".IfcSpatialStructureElement";
    }
    return IfcSpatialStructureElement;
}());
exports.IfcSpatialStructureElement = IfcSpatialStructureElement;
var IfcSpatialStructureElementType = /** @class */ (function () {
    function IfcSpatialStructureElementType(props) {
        this["@class"] = ".IfcSpatialStructureElementType";
    }
    return IfcSpatialStructureElementType;
}());
exports.IfcSpatialStructureElementType = IfcSpatialStructureElementType;
var IfcSpatialZone = /** @class */ (function () {
    function IfcSpatialZone(props) {
        this["@class"] = ".IfcSpatialZone";
    }
    return IfcSpatialZone;
}());
exports.IfcSpatialZone = IfcSpatialZone;
var IfcSpatialZoneType = /** @class */ (function () {
    function IfcSpatialZoneType(props) {
        this["@class"] = ".IfcSpatialZoneType";
    }
    return IfcSpatialZoneType;
}());
exports.IfcSpatialZoneType = IfcSpatialZoneType;
var IfcSphere = /** @class */ (function () {
    function IfcSphere(props) {
        this["@class"] = ".IfcSphere";
    }
    return IfcSphere;
}());
exports.IfcSphere = IfcSphere;
var IfcSphericalSurface = /** @class */ (function () {
    function IfcSphericalSurface(props) {
        this["@class"] = ".IfcSphericalSurface";
    }
    return IfcSphericalSurface;
}());
exports.IfcSphericalSurface = IfcSphericalSurface;
var IfcStackTerminal = /** @class */ (function () {
    function IfcStackTerminal(props) {
        this["@class"] = ".IfcStackTerminal";
    }
    return IfcStackTerminal;
}());
exports.IfcStackTerminal = IfcStackTerminal;
var IfcStackTerminalType = /** @class */ (function () {
    function IfcStackTerminalType(props) {
        this["@class"] = ".IfcStackTerminalType";
    }
    return IfcStackTerminalType;
}());
exports.IfcStackTerminalType = IfcStackTerminalType;
var IfcStair = /** @class */ (function () {
    function IfcStair(props) {
        this["@class"] = ".IfcStair";
    }
    return IfcStair;
}());
exports.IfcStair = IfcStair;
var IfcStairFlight = /** @class */ (function () {
    function IfcStairFlight(props) {
        this["@class"] = ".IfcStairFlight";
    }
    return IfcStairFlight;
}());
exports.IfcStairFlight = IfcStairFlight;
var IfcStairFlightType = /** @class */ (function () {
    function IfcStairFlightType(props) {
        this["@class"] = ".IfcStairFlightType";
    }
    return IfcStairFlightType;
}());
exports.IfcStairFlightType = IfcStairFlightType;
var IfcStairType = /** @class */ (function () {
    function IfcStairType(props) {
        this["@class"] = ".IfcStairType";
    }
    return IfcStairType;
}());
exports.IfcStairType = IfcStairType;
var IfcStructuralAction = /** @class */ (function () {
    function IfcStructuralAction(props) {
        this["@class"] = ".IfcStructuralAction";
    }
    return IfcStructuralAction;
}());
exports.IfcStructuralAction = IfcStructuralAction;
var IfcStructuralConnectionCondition = /** @class */ (function () {
    function IfcStructuralConnectionCondition(props) {
        this["@class"] = ".IfcStructuralConnectionCondition";
    }
    return IfcStructuralConnectionCondition;
}());
exports.IfcStructuralConnectionCondition = IfcStructuralConnectionCondition;
var IfcStructuralCurveAction = /** @class */ (function () {
    function IfcStructuralCurveAction(props) {
        this["@class"] = ".IfcStructuralCurveAction";
    }
    return IfcStructuralCurveAction;
}());
exports.IfcStructuralCurveAction = IfcStructuralCurveAction;
var IfcStructuralCurveMemberVarying = /** @class */ (function () {
    function IfcStructuralCurveMemberVarying(props) {
        this["@class"] = ".IfcStructuralCurveMemberVarying";
    }
    return IfcStructuralCurveMemberVarying;
}());
exports.IfcStructuralCurveMemberVarying = IfcStructuralCurveMemberVarying;
var IfcStructuralCurveReaction = /** @class */ (function () {
    function IfcStructuralCurveReaction(props) {
        this["@class"] = ".IfcStructuralCurveReaction";
    }
    return IfcStructuralCurveReaction;
}());
exports.IfcStructuralCurveReaction = IfcStructuralCurveReaction;
var IfcStructuralItem = /** @class */ (function () {
    function IfcStructuralItem(props) {
        this["@class"] = ".IfcStructuralItem";
    }
    return IfcStructuralItem;
}());
exports.IfcStructuralItem = IfcStructuralItem;
var IfcStructuralLinearAction = /** @class */ (function () {
    function IfcStructuralLinearAction(props) {
        this["@class"] = ".IfcStructuralLinearAction";
    }
    return IfcStructuralLinearAction;
}());
exports.IfcStructuralLinearAction = IfcStructuralLinearAction;
var IfcStructuralLoad = /** @class */ (function () {
    function IfcStructuralLoad(props) {
        this["@class"] = ".IfcStructuralLoad";
    }
    return IfcStructuralLoad;
}());
exports.IfcStructuralLoad = IfcStructuralLoad;
var IfcStructuralLoadCase = /** @class */ (function () {
    function IfcStructuralLoadCase(props) {
        this["@class"] = ".IfcStructuralLoadCase";
    }
    return IfcStructuralLoadCase;
}());
exports.IfcStructuralLoadCase = IfcStructuralLoadCase;
var IfcStructuralLoadGroup = /** @class */ (function () {
    function IfcStructuralLoadGroup(props) {
        this["@class"] = ".IfcStructuralLoadGroup";
    }
    return IfcStructuralLoadGroup;
}());
exports.IfcStructuralLoadGroup = IfcStructuralLoadGroup;
var IfcStructuralLoadLinearForce = /** @class */ (function () {
    function IfcStructuralLoadLinearForce(props) {
        this["@class"] = ".IfcStructuralLoadLinearForce";
    }
    return IfcStructuralLoadLinearForce;
}());
exports.IfcStructuralLoadLinearForce = IfcStructuralLoadLinearForce;
var IfcStructuralLoadOrResult = /** @class */ (function () {
    function IfcStructuralLoadOrResult(props) {
        this["@class"] = ".IfcStructuralLoadOrResult";
    }
    return IfcStructuralLoadOrResult;
}());
exports.IfcStructuralLoadOrResult = IfcStructuralLoadOrResult;
var IfcStructuralLoadPlanarForce = /** @class */ (function () {
    function IfcStructuralLoadPlanarForce(props) {
        this["@class"] = ".IfcStructuralLoadPlanarForce";
    }
    return IfcStructuralLoadPlanarForce;
}());
exports.IfcStructuralLoadPlanarForce = IfcStructuralLoadPlanarForce;
var IfcStructuralLoadSingleDisplacement = /** @class */ (function () {
    function IfcStructuralLoadSingleDisplacement(props) {
        this["@class"] = ".IfcStructuralLoadSingleDisplacement";
    }
    return IfcStructuralLoadSingleDisplacement;
}());
exports.IfcStructuralLoadSingleDisplacement = IfcStructuralLoadSingleDisplacement;
var IfcStructuralLoadSingleDisplacementDistortion = /** @class */ (function () {
    function IfcStructuralLoadSingleDisplacementDistortion(props) {
        this["@class"] = ".IfcStructuralLoadSingleDisplacementDistortion";
    }
    return IfcStructuralLoadSingleDisplacementDistortion;
}());
exports.IfcStructuralLoadSingleDisplacementDistortion = IfcStructuralLoadSingleDisplacementDistortion;
var IfcStructuralLoadSingleForce = /** @class */ (function () {
    function IfcStructuralLoadSingleForce(props) {
        this["@class"] = ".IfcStructuralLoadSingleForce";
    }
    return IfcStructuralLoadSingleForce;
}());
exports.IfcStructuralLoadSingleForce = IfcStructuralLoadSingleForce;
var IfcStructuralLoadSingleForceWarping = /** @class */ (function () {
    function IfcStructuralLoadSingleForceWarping(props) {
        this["@class"] = ".IfcStructuralLoadSingleForceWarping";
    }
    return IfcStructuralLoadSingleForceWarping;
}());
exports.IfcStructuralLoadSingleForceWarping = IfcStructuralLoadSingleForceWarping;
var IfcStructuralLoadStatic = /** @class */ (function () {
    function IfcStructuralLoadStatic(props) {
        this["@class"] = ".IfcStructuralLoadStatic";
    }
    return IfcStructuralLoadStatic;
}());
exports.IfcStructuralLoadStatic = IfcStructuralLoadStatic;
var IfcStructuralLoadTemperature = /** @class */ (function () {
    function IfcStructuralLoadTemperature(props) {
        this["@class"] = ".IfcStructuralLoadTemperature";
    }
    return IfcStructuralLoadTemperature;
}());
exports.IfcStructuralLoadTemperature = IfcStructuralLoadTemperature;
var IfcStructuralMember = /** @class */ (function () {
    function IfcStructuralMember(props) {
        this["@class"] = ".IfcStructuralMember";
    }
    return IfcStructuralMember;
}());
exports.IfcStructuralMember = IfcStructuralMember;
var IfcStructuralPlanarAction = /** @class */ (function () {
    function IfcStructuralPlanarAction(props) {
        this["@class"] = ".IfcStructuralPlanarAction";
    }
    return IfcStructuralPlanarAction;
}());
exports.IfcStructuralPlanarAction = IfcStructuralPlanarAction;
var IfcStructuralPointAction = /** @class */ (function () {
    function IfcStructuralPointAction(props) {
        this["@class"] = ".IfcStructuralPointAction";
    }
    return IfcStructuralPointAction;
}());
exports.IfcStructuralPointAction = IfcStructuralPointAction;
var IfcStructuralPointReaction = /** @class */ (function () {
    function IfcStructuralPointReaction(props) {
        this["@class"] = ".IfcStructuralPointReaction";
    }
    return IfcStructuralPointReaction;
}());
exports.IfcStructuralPointReaction = IfcStructuralPointReaction;
var IfcStructuralReaction = /** @class */ (function () {
    function IfcStructuralReaction(props) {
        this["@class"] = ".IfcStructuralReaction";
    }
    return IfcStructuralReaction;
}());
exports.IfcStructuralReaction = IfcStructuralReaction;
var IfcStructuralSurfaceAction = /** @class */ (function () {
    function IfcStructuralSurfaceAction(props) {
        this["@class"] = ".IfcStructuralSurfaceAction";
    }
    return IfcStructuralSurfaceAction;
}());
exports.IfcStructuralSurfaceAction = IfcStructuralSurfaceAction;
var IfcStructuralSurfaceConnection = /** @class */ (function () {
    function IfcStructuralSurfaceConnection(props) {
        this["@class"] = ".IfcStructuralSurfaceConnection";
    }
    return IfcStructuralSurfaceConnection;
}());
exports.IfcStructuralSurfaceConnection = IfcStructuralSurfaceConnection;
var IfcStructuralSurfaceMember = /** @class */ (function () {
    function IfcStructuralSurfaceMember(props) {
        this["@class"] = ".IfcStructuralSurfaceMember";
    }
    return IfcStructuralSurfaceMember;
}());
exports.IfcStructuralSurfaceMember = IfcStructuralSurfaceMember;
var IfcStructuralSurfaceMemberVarying = /** @class */ (function () {
    function IfcStructuralSurfaceMemberVarying(props) {
        this["@class"] = ".IfcStructuralSurfaceMemberVarying";
    }
    return IfcStructuralSurfaceMemberVarying;
}());
exports.IfcStructuralSurfaceMemberVarying = IfcStructuralSurfaceMemberVarying;
var IfcStructuralSurfaceReaction = /** @class */ (function () {
    function IfcStructuralSurfaceReaction(props) {
        this["@class"] = ".IfcStructuralSurfaceReaction";
    }
    return IfcStructuralSurfaceReaction;
}());
exports.IfcStructuralSurfaceReaction = IfcStructuralSurfaceReaction;
var IfcStyleModel = /** @class */ (function () {
    function IfcStyleModel(props) {
        this["@class"] = ".IfcStyleModel";
    }
    return IfcStyleModel;
}());
exports.IfcStyleModel = IfcStyleModel;
var IfcStyledRepresentation = /** @class */ (function () {
    function IfcStyledRepresentation(props) {
        this["@class"] = ".IfcStyledRepresentation";
    }
    return IfcStyledRepresentation;
}());
exports.IfcStyledRepresentation = IfcStyledRepresentation;
var IfcSubContractResource = /** @class */ (function () {
    function IfcSubContractResource(props) {
        this["@class"] = ".IfcSubContractResource";
    }
    return IfcSubContractResource;
}());
exports.IfcSubContractResource = IfcSubContractResource;
var IfcSubContractResourceType = /** @class */ (function () {
    function IfcSubContractResourceType(props) {
        this["@class"] = ".IfcSubContractResourceType";
    }
    return IfcSubContractResourceType;
}());
exports.IfcSubContractResourceType = IfcSubContractResourceType;
var IfcSurface = /** @class */ (function () {
    function IfcSurface(props) {
        this["@class"] = ".IfcSurface";
    }
    return IfcSurface;
}());
exports.IfcSurface = IfcSurface;
var AssociatedGeometry = /** @class */ (function () {
    function AssociatedGeometry(props) {
        this["@class"] = ".AssociatedGeometry";
        if (props) {
            this.IfcPcurve = (props.IfcPcurve) ? new IfcPcurve(props.IfcPcurve) : undefined;
        }
    }
    return AssociatedGeometry;
}());
exports.AssociatedGeometry = AssociatedGeometry;
var IfcSurfaceFeature = /** @class */ (function () {
    function IfcSurfaceFeature(props) {
        this["@class"] = ".IfcSurfaceFeature";
    }
    return IfcSurfaceFeature;
}());
exports.IfcSurfaceFeature = IfcSurfaceFeature;
var IfcSurfaceReinforcementArea = /** @class */ (function () {
    function IfcSurfaceReinforcementArea(props) {
        this["@class"] = ".IfcSurfaceReinforcementArea";
    }
    return IfcSurfaceReinforcementArea;
}());
exports.IfcSurfaceReinforcementArea = IfcSurfaceReinforcementArea;
var IfcSurfaceStyleRefraction = /** @class */ (function () {
    function IfcSurfaceStyleRefraction(props) {
        this["@class"] = ".IfcSurfaceStyleRefraction";
    }
    return IfcSurfaceStyleRefraction;
}());
exports.IfcSurfaceStyleRefraction = IfcSurfaceStyleRefraction;
var IfcSweptDiskSolidPolygonal = /** @class */ (function () {
    function IfcSweptDiskSolidPolygonal(props) {
        this["@class"] = ".IfcSweptDiskSolidPolygonal";
    }
    return IfcSweptDiskSolidPolygonal;
}());
exports.IfcSweptDiskSolidPolygonal = IfcSweptDiskSolidPolygonal;
var IfcSwitchingDevice = /** @class */ (function () {
    function IfcSwitchingDevice(props) {
        this["@class"] = ".IfcSwitchingDevice";
    }
    return IfcSwitchingDevice;
}());
exports.IfcSwitchingDevice = IfcSwitchingDevice;
var IfcSwitchingDeviceType = /** @class */ (function () {
    function IfcSwitchingDeviceType(props) {
        this["@class"] = ".IfcSwitchingDeviceType";
    }
    return IfcSwitchingDeviceType;
}());
exports.IfcSwitchingDeviceType = IfcSwitchingDeviceType;
var IfcSystem = /** @class */ (function () {
    function IfcSystem(props) {
        this["@class"] = ".IfcSystem";
    }
    return IfcSystem;
}());
exports.IfcSystem = IfcSystem;
var IfcSystemFurnitureElement = /** @class */ (function () {
    function IfcSystemFurnitureElement(props) {
        this["@class"] = ".IfcSystemFurnitureElement";
    }
    return IfcSystemFurnitureElement;
}());
exports.IfcSystemFurnitureElement = IfcSystemFurnitureElement;
var IfcSystemFurnitureElementType = /** @class */ (function () {
    function IfcSystemFurnitureElementType(props) {
        this["@class"] = ".IfcSystemFurnitureElementType";
    }
    return IfcSystemFurnitureElementType;
}());
exports.IfcSystemFurnitureElementType = IfcSystemFurnitureElementType;
var IfcTShapeProfileDef = /** @class */ (function () {
    function IfcTShapeProfileDef(props) {
        this["@class"] = ".IfcTShapeProfileDef";
    }
    return IfcTShapeProfileDef;
}());
exports.IfcTShapeProfileDef = IfcTShapeProfileDef;
var IfcTank = /** @class */ (function () {
    function IfcTank(props) {
        this["@class"] = ".IfcTank";
    }
    return IfcTank;
}());
exports.IfcTank = IfcTank;
var IfcTankType = /** @class */ (function () {
    function IfcTankType(props) {
        this["@class"] = ".IfcTankType";
    }
    return IfcTankType;
}());
exports.IfcTankType = IfcTankType;
var IfcTaskTime = /** @class */ (function () {
    function IfcTaskTime(props) {
        this["@class"] = ".IfcTaskTime";
    }
    return IfcTaskTime;
}());
exports.IfcTaskTime = IfcTaskTime;
var IfcTaskType = /** @class */ (function () {
    function IfcTaskType(props) {
        this["@class"] = ".IfcTaskType";
    }
    return IfcTaskType;
}());
exports.IfcTaskType = IfcTaskType;
var IfcTelecomAddress = /** @class */ (function () {
    function IfcTelecomAddress(props) {
        this["@class"] = ".IfcTelecomAddress";
    }
    return IfcTelecomAddress;
}());
exports.IfcTelecomAddress = IfcTelecomAddress;
var IfcTendon = /** @class */ (function () {
    function IfcTendon(props) {
        this["@class"] = ".IfcTendon";
    }
    return IfcTendon;
}());
exports.IfcTendon = IfcTendon;
var IfcTendonAnchor = /** @class */ (function () {
    function IfcTendonAnchor(props) {
        this["@class"] = ".IfcTendonAnchor";
    }
    return IfcTendonAnchor;
}());
exports.IfcTendonAnchor = IfcTendonAnchor;
var IfcTendonAnchorType = /** @class */ (function () {
    function IfcTendonAnchorType(props) {
        this["@class"] = ".IfcTendonAnchorType";
    }
    return IfcTendonAnchorType;
}());
exports.IfcTendonAnchorType = IfcTendonAnchorType;
var IfcTendonType = /** @class */ (function () {
    function IfcTendonType(props) {
        this["@class"] = ".IfcTendonType";
    }
    return IfcTendonType;
}());
exports.IfcTendonType = IfcTendonType;
var IfcTessellatedItem = /** @class */ (function () {
    function IfcTessellatedItem(props) {
        this["@class"] = ".IfcTessellatedItem";
    }
    return IfcTessellatedItem;
}());
exports.IfcTessellatedItem = IfcTessellatedItem;
var IfcTextureCoordinateGenerator = /** @class */ (function () {
    function IfcTextureCoordinateGenerator(props) {
        this["@class"] = ".IfcTextureCoordinateGenerator";
    }
    return IfcTextureCoordinateGenerator;
}());
exports.IfcTextureCoordinateGenerator = IfcTextureCoordinateGenerator;
var IfcTextureVertex = /** @class */ (function () {
    function IfcTextureVertex(props) {
        this["@class"] = ".IfcTextureVertex";
    }
    return IfcTextureVertex;
}());
exports.IfcTextureVertex = IfcTextureVertex;
var IfcTextureVertexList = /** @class */ (function () {
    function IfcTextureVertexList(props) {
        this["@class"] = ".IfcTextureVertexList";
    }
    return IfcTextureVertexList;
}());
exports.IfcTextureVertexList = IfcTextureVertexList;
var IfcTimePeriod = /** @class */ (function () {
    function IfcTimePeriod(props) {
        this["@class"] = ".IfcTimePeriod";
    }
    return IfcTimePeriod;
}());
exports.IfcTimePeriod = IfcTimePeriod;
var IfcTopologicalRepresentationItem = /** @class */ (function () {
    function IfcTopologicalRepresentationItem(props) {
        this["@class"] = ".IfcTopologicalRepresentationItem";
    }
    return IfcTopologicalRepresentationItem;
}());
exports.IfcTopologicalRepresentationItem = IfcTopologicalRepresentationItem;
var IfcTopologyRepresentation = /** @class */ (function () {
    function IfcTopologyRepresentation(props) {
        this["@class"] = ".IfcTopologyRepresentation";
    }
    return IfcTopologyRepresentation;
}());
exports.IfcTopologyRepresentation = IfcTopologyRepresentation;
var IfcToroidalSurface = /** @class */ (function () {
    function IfcToroidalSurface(props) {
        this["@class"] = ".IfcToroidalSurface";
    }
    return IfcToroidalSurface;
}());
exports.IfcToroidalSurface = IfcToroidalSurface;
var IfcTransformer = /** @class */ (function () {
    function IfcTransformer(props) {
        this["@class"] = ".IfcTransformer";
    }
    return IfcTransformer;
}());
exports.IfcTransformer = IfcTransformer;
var IfcTransformerType = /** @class */ (function () {
    function IfcTransformerType(props) {
        this["@class"] = ".IfcTransformerType";
    }
    return IfcTransformerType;
}());
exports.IfcTransformerType = IfcTransformerType;
var IfcTransportElement = /** @class */ (function () {
    function IfcTransportElement(props) {
        this["@class"] = ".IfcTransportElement";
    }
    return IfcTransportElement;
}());
exports.IfcTransportElement = IfcTransportElement;
var IfcTransportElementType = /** @class */ (function () {
    function IfcTransportElementType(props) {
        this["@class"] = ".IfcTransportElementType";
    }
    return IfcTransportElementType;
}());
exports.IfcTransportElementType = IfcTransportElementType;
var IfcTrapeziumProfileDef = /** @class */ (function () {
    function IfcTrapeziumProfileDef(props) {
        this["@class"] = ".IfcTrapeziumProfileDef";
    }
    return IfcTrapeziumProfileDef;
}());
exports.IfcTrapeziumProfileDef = IfcTrapeziumProfileDef;
var IfcTriangulatedFaceSet = /** @class */ (function () {
    function IfcTriangulatedFaceSet(props) {
        this["@class"] = ".IfcTriangulatedFaceSet";
    }
    return IfcTriangulatedFaceSet;
}());
exports.IfcTriangulatedFaceSet = IfcTriangulatedFaceSet;
var IfcTubeBundle = /** @class */ (function () {
    function IfcTubeBundle(props) {
        this["@class"] = ".IfcTubeBundle";
    }
    return IfcTubeBundle;
}());
exports.IfcTubeBundle = IfcTubeBundle;
var IfcTubeBundleType = /** @class */ (function () {
    function IfcTubeBundleType(props) {
        this["@class"] = ".IfcTubeBundleType";
    }
    return IfcTubeBundleType;
}());
exports.IfcTubeBundleType = IfcTubeBundleType;
var IfcTypeProcess = /** @class */ (function () {
    function IfcTypeProcess(props) {
        this["@class"] = ".IfcTypeProcess";
    }
    return IfcTypeProcess;
}());
exports.IfcTypeProcess = IfcTypeProcess;
var IfcTypeResource = /** @class */ (function () {
    function IfcTypeResource(props) {
        this["@class"] = ".IfcTypeResource";
    }
    return IfcTypeResource;
}());
exports.IfcTypeResource = IfcTypeResource;
var IfcUShapeProfileDef = /** @class */ (function () {
    function IfcUShapeProfileDef(props) {
        this["@class"] = ".IfcUShapeProfileDef";
    }
    return IfcUShapeProfileDef;
}());
exports.IfcUShapeProfileDef = IfcUShapeProfileDef;
var IfcUnitaryControlElement = /** @class */ (function () {
    function IfcUnitaryControlElement(props) {
        this["@class"] = ".IfcUnitaryControlElement";
    }
    return IfcUnitaryControlElement;
}());
exports.IfcUnitaryControlElement = IfcUnitaryControlElement;
var IfcUnitaryControlElementType = /** @class */ (function () {
    function IfcUnitaryControlElementType(props) {
        this["@class"] = ".IfcUnitaryControlElementType";
    }
    return IfcUnitaryControlElementType;
}());
exports.IfcUnitaryControlElementType = IfcUnitaryControlElementType;
var IfcUnitaryEquipment = /** @class */ (function () {
    function IfcUnitaryEquipment(props) {
        this["@class"] = ".IfcUnitaryEquipment";
    }
    return IfcUnitaryEquipment;
}());
exports.IfcUnitaryEquipment = IfcUnitaryEquipment;
var IfcUnitaryEquipmentType = /** @class */ (function () {
    function IfcUnitaryEquipmentType(props) {
        this["@class"] = ".IfcUnitaryEquipmentType";
    }
    return IfcUnitaryEquipmentType;
}());
exports.IfcUnitaryEquipmentType = IfcUnitaryEquipmentType;
var IfcValve = /** @class */ (function () {
    function IfcValve(props) {
        this["@class"] = ".IfcValve";
    }
    return IfcValve;
}());
exports.IfcValve = IfcValve;
var IfcValveType = /** @class */ (function () {
    function IfcValveType(props) {
        this["@class"] = ".IfcValveType";
    }
    return IfcValveType;
}());
exports.IfcValveType = IfcValveType;
var IfcVertex = /** @class */ (function () {
    function IfcVertex(props) {
        this["@class"] = ".IfcVertex";
    }
    return IfcVertex;
}());
exports.IfcVertex = IfcVertex;
var IfcVibrationIsolator = /** @class */ (function () {
    function IfcVibrationIsolator(props) {
        this["@class"] = ".IfcVibrationIsolator";
    }
    return IfcVibrationIsolator;
}());
exports.IfcVibrationIsolator = IfcVibrationIsolator;
var IfcVibrationIsolatorType = /** @class */ (function () {
    function IfcVibrationIsolatorType(props) {
        this["@class"] = ".IfcVibrationIsolatorType";
    }
    return IfcVibrationIsolatorType;
}());
exports.IfcVibrationIsolatorType = IfcVibrationIsolatorType;
var IfcVirtualElement = /** @class */ (function () {
    function IfcVirtualElement(props) {
        this["@class"] = ".IfcVirtualElement";
    }
    return IfcVirtualElement;
}());
exports.IfcVirtualElement = IfcVirtualElement;
var IntersectingAxes = /** @class */ (function () {
    function IntersectingAxes(props) {
        this["@class"] = ".IntersectingAxes";
        if (props) {
            this.IfcGridAxis = (props.IfcGridAxis) ? new IfcGridAxis(props.IfcGridAxis) : undefined;
        }
    }
    return IntersectingAxes;
}());
exports.IntersectingAxes = IntersectingAxes;
var IfcVoidingFeature = /** @class */ (function () {
    function IfcVoidingFeature(props) {
        this["@class"] = ".IfcVoidingFeature";
    }
    return IfcVoidingFeature;
}());
exports.IfcVoidingFeature = IfcVoidingFeature;
var IfcWall = /** @class */ (function () {
    function IfcWall(props) {
        this["@class"] = ".IfcWall";
    }
    return IfcWall;
}());
exports.IfcWall = IfcWall;
var IfcWallElementedCase = /** @class */ (function () {
    function IfcWallElementedCase(props) {
        this["@class"] = ".IfcWallElementedCase";
    }
    return IfcWallElementedCase;
}());
exports.IfcWallElementedCase = IfcWallElementedCase;
var IfcWallStandardCase = /** @class */ (function () {
    function IfcWallStandardCase(props) {
        this["@class"] = ".IfcWallStandardCase";
    }
    return IfcWallStandardCase;
}());
exports.IfcWallStandardCase = IfcWallStandardCase;
var IfcWallType = /** @class */ (function () {
    function IfcWallType(props) {
        this["@class"] = ".IfcWallType";
    }
    return IfcWallType;
}());
exports.IfcWallType = IfcWallType;
var IfcWasteTerminal = /** @class */ (function () {
    function IfcWasteTerminal(props) {
        this["@class"] = ".IfcWasteTerminal";
    }
    return IfcWasteTerminal;
}());
exports.IfcWasteTerminal = IfcWasteTerminal;
var IfcWasteTerminalType = /** @class */ (function () {
    function IfcWasteTerminalType(props) {
        this["@class"] = ".IfcWasteTerminalType";
    }
    return IfcWasteTerminalType;
}());
exports.IfcWasteTerminalType = IfcWasteTerminalType;
var IfcWindow = /** @class */ (function () {
    function IfcWindow(props) {
        this["@class"] = ".IfcWindow";
    }
    return IfcWindow;
}());
exports.IfcWindow = IfcWindow;
var IfcWindowStandardCase = /** @class */ (function () {
    function IfcWindowStandardCase(props) {
        this["@class"] = ".IfcWindowStandardCase";
    }
    return IfcWindowStandardCase;
}());
exports.IfcWindowStandardCase = IfcWindowStandardCase;
var IfcWindowStyle = /** @class */ (function () {
    function IfcWindowStyle(props) {
        this["@class"] = ".IfcWindowStyle";
    }
    return IfcWindowStyle;
}());
exports.IfcWindowStyle = IfcWindowStyle;
var IfcWindowType = /** @class */ (function () {
    function IfcWindowType(props) {
        this["@class"] = ".IfcWindowType";
    }
    return IfcWindowType;
}());
exports.IfcWindowType = IfcWindowType;
var IfcWorkPlan = /** @class */ (function () {
    function IfcWorkPlan(props) {
        this["@class"] = ".IfcWorkPlan";
    }
    return IfcWorkPlan;
}());
exports.IfcWorkPlan = IfcWorkPlan;
var IfcWorkSchedule = /** @class */ (function () {
    function IfcWorkSchedule(props) {
        this["@class"] = ".IfcWorkSchedule";
    }
    return IfcWorkSchedule;
}());
exports.IfcWorkSchedule = IfcWorkSchedule;
var IfcZShapeProfileDef = /** @class */ (function () {
    function IfcZShapeProfileDef(props) {
        this["@class"] = ".IfcZShapeProfileDef";
    }
    return IfcZShapeProfileDef;
}());
exports.IfcZShapeProfileDef = IfcZShapeProfileDef;
var IfcZone = /** @class */ (function () {
    function IfcZone(props) {
        this["@class"] = ".IfcZone";
    }
    return IfcZone;
}());
exports.IfcZone = IfcZone;
var IfcArcIndex = /** @class */ (function () {
    function IfcArcIndex(props) {
        this["@class"] = ".IfcArcIndex";
    }
    return IfcArcIndex;
}());
exports.IfcArcIndex = IfcArcIndex;
var IfcBinary = /** @class */ (function () {
    function IfcBinary(props) {
        this["@class"] = ".IfcBinary";
    }
    return IfcBinary;
}());
exports.IfcBinary = IfcBinary;
var IfcComplexNumber = /** @class */ (function () {
    function IfcComplexNumber(props) {
        this["@class"] = ".IfcComplexNumber";
    }
    return IfcComplexNumber;
}());
exports.IfcComplexNumber = IfcComplexNumber;
var IfcCompoundPlaneAngleMeasure = /** @class */ (function () {
    function IfcCompoundPlaneAngleMeasure(props) {
        this["@class"] = ".IfcCompoundPlaneAngleMeasure";
    }
    return IfcCompoundPlaneAngleMeasure;
}());
exports.IfcCompoundPlaneAngleMeasure = IfcCompoundPlaneAngleMeasure;
var IfcLineIndex = /** @class */ (function () {
    function IfcLineIndex(props) {
        this["@class"] = ".IfcLineIndex";
    }
    return IfcLineIndex;
}());
exports.IfcLineIndex = IfcLineIndex;
var IfcPropertySetDefinitionSet = /** @class */ (function () {
    function IfcPropertySetDefinitionSet(props) {
        var _a;
        this["@class"] = ".IfcPropertySetDefinitionSet";
        if (props) {
            this.IfcPropertySetDefinition = (_a = props.IfcPropertySetDefinition) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPropertySetDefinition(o); });
            this.$undefined = props.$undefined;
            this.$undefined = props.$undefined;
            this.$undefined = props.$undefined;
        }
    }
    return IfcPropertySetDefinitionSet;
}());
exports.IfcPropertySetDefinitionSet = IfcPropertySetDefinitionSet;
var Entity = /** @class */ (function () {
    function Entity(props) {
        this["@class"] = ".Entity";
        if (props) {
            this.$href = props.$href;
            this.$ref = props.$ref;
        }
    }
    return Entity;
}());
exports.Entity = Entity;
var HexBinary = /** @class */ (function () {
    function HexBinary(props) {
        this["@class"] = ".HexBinary";
    }
    return HexBinary;
}());
exports.HexBinary = HexBinary;
var IfcAbsorbedDoseMeasure = /** @class */ (function () {
    function IfcAbsorbedDoseMeasure() {
    }
    return IfcAbsorbedDoseMeasure;
}());
exports.IfcAbsorbedDoseMeasure = IfcAbsorbedDoseMeasure;
-wrapper;
{
    constructor(props ?  : IfcAbsorbedDoseMeasure - wrapper);
    {
        this["@class"] = ".IfcAbsorbedDoseMeasure-wrapper";
    }
}
var IfcAccelerationMeasure = /** @class */ (function () {
    function IfcAccelerationMeasure() {
    }
    return IfcAccelerationMeasure;
}());
exports.IfcAccelerationMeasure = IfcAccelerationMeasure;
-wrapper;
{
    constructor(props ?  : IfcAccelerationMeasure - wrapper);
    {
        this["@class"] = ".IfcAccelerationMeasure-wrapper";
    }
}
var IfcAmountOfSubstanceMeasure = /** @class */ (function () {
    function IfcAmountOfSubstanceMeasure() {
    }
    return IfcAmountOfSubstanceMeasure;
}());
exports.IfcAmountOfSubstanceMeasure = IfcAmountOfSubstanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcAmountOfSubstanceMeasure - wrapper);
    {
        this["@class"] = ".IfcAmountOfSubstanceMeasure-wrapper";
    }
}
var IfcAngularVelocityMeasure = /** @class */ (function () {
    function IfcAngularVelocityMeasure() {
    }
    return IfcAngularVelocityMeasure;
}());
exports.IfcAngularVelocityMeasure = IfcAngularVelocityMeasure;
-wrapper;
{
    constructor(props ?  : IfcAngularVelocityMeasure - wrapper);
    {
        this["@class"] = ".IfcAngularVelocityMeasure-wrapper";
    }
}
var IfcArcIndex = /** @class */ (function () {
    function IfcArcIndex() {
    }
    return IfcArcIndex;
}());
exports.IfcArcIndex = IfcArcIndex;
-wrapper;
{
    constructor(props ?  : IfcArcIndex - wrapper);
    {
        this["@class"] = ".IfcArcIndex-wrapper";
    }
}
var IfcAreaDensityMeasure = /** @class */ (function () {
    function IfcAreaDensityMeasure() {
    }
    return IfcAreaDensityMeasure;
}());
exports.IfcAreaDensityMeasure = IfcAreaDensityMeasure;
-wrapper;
{
    constructor(props ?  : IfcAreaDensityMeasure - wrapper);
    {
        this["@class"] = ".IfcAreaDensityMeasure-wrapper";
    }
}
var IfcAreaMeasure = /** @class */ (function () {
    function IfcAreaMeasure() {
    }
    return IfcAreaMeasure;
}());
exports.IfcAreaMeasure = IfcAreaMeasure;
-wrapper;
{
    constructor(props ?  : IfcAreaMeasure - wrapper);
    {
        this["@class"] = ".IfcAreaMeasure-wrapper";
    }
}
var IfcBinary = /** @class */ (function () {
    function IfcBinary() {
    }
    return IfcBinary;
}());
exports.IfcBinary = IfcBinary;
-wrapper;
{
    constructor(props ?  : IfcBinary - wrapper);
    {
        this["@class"] = ".IfcBinary-wrapper";
    }
}
var IfcBoolean = /** @class */ (function () {
    function IfcBoolean() {
    }
    return IfcBoolean;
}());
exports.IfcBoolean = IfcBoolean;
-wrapper;
{
    constructor(props ?  : IfcBoolean - wrapper);
    {
        this["@class"] = ".IfcBoolean-wrapper";
    }
}
var IfcComplexNumber = /** @class */ (function () {
    function IfcComplexNumber() {
    }
    return IfcComplexNumber;
}());
exports.IfcComplexNumber = IfcComplexNumber;
-wrapper;
{
    constructor(props ?  : IfcComplexNumber - wrapper);
    {
        this["@class"] = ".IfcComplexNumber-wrapper";
    }
}
var IfcCompoundPlaneAngleMeasure = /** @class */ (function () {
    function IfcCompoundPlaneAngleMeasure() {
    }
    return IfcCompoundPlaneAngleMeasure;
}());
exports.IfcCompoundPlaneAngleMeasure = IfcCompoundPlaneAngleMeasure;
-wrapper;
{
    constructor(props ?  : IfcCompoundPlaneAngleMeasure - wrapper);
    {
        this["@class"] = ".IfcCompoundPlaneAngleMeasure-wrapper";
    }
}
var IfcContextDependentMeasure = /** @class */ (function () {
    function IfcContextDependentMeasure() {
    }
    return IfcContextDependentMeasure;
}());
exports.IfcContextDependentMeasure = IfcContextDependentMeasure;
-wrapper;
{
    constructor(props ?  : IfcContextDependentMeasure - wrapper);
    {
        this["@class"] = ".IfcContextDependentMeasure-wrapper";
    }
}
var IfcCountMeasure = /** @class */ (function () {
    function IfcCountMeasure() {
    }
    return IfcCountMeasure;
}());
exports.IfcCountMeasure = IfcCountMeasure;
-wrapper;
{
    constructor(props ?  : IfcCountMeasure - wrapper);
    {
        this["@class"] = ".IfcCountMeasure-wrapper";
    }
}
var IfcCurvatureMeasure = /** @class */ (function () {
    function IfcCurvatureMeasure() {
    }
    return IfcCurvatureMeasure;
}());
exports.IfcCurvatureMeasure = IfcCurvatureMeasure;
-wrapper;
{
    constructor(props ?  : IfcCurvatureMeasure - wrapper);
    {
        this["@class"] = ".IfcCurvatureMeasure-wrapper";
    }
}
var IfcDate = /** @class */ (function () {
    function IfcDate() {
    }
    return IfcDate;
}());
exports.IfcDate = IfcDate;
-wrapper;
{
    constructor(props ?  : IfcDate - wrapper);
    {
        this["@class"] = ".IfcDate-wrapper";
    }
}
var IfcDateTime = /** @class */ (function () {
    function IfcDateTime() {
    }
    return IfcDateTime;
}());
exports.IfcDateTime = IfcDateTime;
-wrapper;
{
    constructor(props ?  : IfcDateTime - wrapper);
    {
        this["@class"] = ".IfcDateTime-wrapper";
    }
}
var IfcDescriptiveMeasure = /** @class */ (function () {
    function IfcDescriptiveMeasure() {
    }
    return IfcDescriptiveMeasure;
}());
exports.IfcDescriptiveMeasure = IfcDescriptiveMeasure;
-wrapper;
{
    constructor(props ?  : IfcDescriptiveMeasure - wrapper);
    {
        this["@class"] = ".IfcDescriptiveMeasure-wrapper";
    }
}
var IfcDoseEquivalentMeasure = /** @class */ (function () {
    function IfcDoseEquivalentMeasure() {
    }
    return IfcDoseEquivalentMeasure;
}());
exports.IfcDoseEquivalentMeasure = IfcDoseEquivalentMeasure;
-wrapper;
{
    constructor(props ?  : IfcDoseEquivalentMeasure - wrapper);
    {
        this["@class"] = ".IfcDoseEquivalentMeasure-wrapper";
    }
}
var IfcDuration = /** @class */ (function () {
    function IfcDuration() {
    }
    return IfcDuration;
}());
exports.IfcDuration = IfcDuration;
-wrapper;
{
    constructor(props ?  : IfcDuration - wrapper);
    {
        this["@class"] = ".IfcDuration-wrapper";
    }
}
var IfcDynamicViscosityMeasure = /** @class */ (function () {
    function IfcDynamicViscosityMeasure() {
    }
    return IfcDynamicViscosityMeasure;
}());
exports.IfcDynamicViscosityMeasure = IfcDynamicViscosityMeasure;
-wrapper;
{
    constructor(props ?  : IfcDynamicViscosityMeasure - wrapper);
    {
        this["@class"] = ".IfcDynamicViscosityMeasure-wrapper";
    }
}
var IfcElectricCapacitanceMeasure = /** @class */ (function () {
    function IfcElectricCapacitanceMeasure() {
    }
    return IfcElectricCapacitanceMeasure;
}());
exports.IfcElectricCapacitanceMeasure = IfcElectricCapacitanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricCapacitanceMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricCapacitanceMeasure-wrapper";
    }
}
var IfcElectricChargeMeasure = /** @class */ (function () {
    function IfcElectricChargeMeasure() {
    }
    return IfcElectricChargeMeasure;
}());
exports.IfcElectricChargeMeasure = IfcElectricChargeMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricChargeMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricChargeMeasure-wrapper";
    }
}
var IfcElectricConductanceMeasure = /** @class */ (function () {
    function IfcElectricConductanceMeasure() {
    }
    return IfcElectricConductanceMeasure;
}());
exports.IfcElectricConductanceMeasure = IfcElectricConductanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricConductanceMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricConductanceMeasure-wrapper";
    }
}
var IfcElectricCurrentMeasure = /** @class */ (function () {
    function IfcElectricCurrentMeasure() {
    }
    return IfcElectricCurrentMeasure;
}());
exports.IfcElectricCurrentMeasure = IfcElectricCurrentMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricCurrentMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricCurrentMeasure-wrapper";
    }
}
var IfcElectricResistanceMeasure = /** @class */ (function () {
    function IfcElectricResistanceMeasure() {
    }
    return IfcElectricResistanceMeasure;
}());
exports.IfcElectricResistanceMeasure = IfcElectricResistanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricResistanceMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricResistanceMeasure-wrapper";
    }
}
var IfcElectricVoltageMeasure = /** @class */ (function () {
    function IfcElectricVoltageMeasure() {
    }
    return IfcElectricVoltageMeasure;
}());
exports.IfcElectricVoltageMeasure = IfcElectricVoltageMeasure;
-wrapper;
{
    constructor(props ?  : IfcElectricVoltageMeasure - wrapper);
    {
        this["@class"] = ".IfcElectricVoltageMeasure-wrapper";
    }
}
var IfcEnergyMeasure = /** @class */ (function () {
    function IfcEnergyMeasure() {
    }
    return IfcEnergyMeasure;
}());
exports.IfcEnergyMeasure = IfcEnergyMeasure;
-wrapper;
{
    constructor(props ?  : IfcEnergyMeasure - wrapper);
    {
        this["@class"] = ".IfcEnergyMeasure-wrapper";
    }
}
var IfcForceMeasure = /** @class */ (function () {
    function IfcForceMeasure() {
    }
    return IfcForceMeasure;
}());
exports.IfcForceMeasure = IfcForceMeasure;
-wrapper;
{
    constructor(props ?  : IfcForceMeasure - wrapper);
    {
        this["@class"] = ".IfcForceMeasure-wrapper";
    }
}
var IfcFrequencyMeasure = /** @class */ (function () {
    function IfcFrequencyMeasure() {
    }
    return IfcFrequencyMeasure;
}());
exports.IfcFrequencyMeasure = IfcFrequencyMeasure;
-wrapper;
{
    constructor(props ?  : IfcFrequencyMeasure - wrapper);
    {
        this["@class"] = ".IfcFrequencyMeasure-wrapper";
    }
}
var IfcHeatFluxDensityMeasure = /** @class */ (function () {
    function IfcHeatFluxDensityMeasure() {
    }
    return IfcHeatFluxDensityMeasure;
}());
exports.IfcHeatFluxDensityMeasure = IfcHeatFluxDensityMeasure;
-wrapper;
{
    constructor(props ?  : IfcHeatFluxDensityMeasure - wrapper);
    {
        this["@class"] = ".IfcHeatFluxDensityMeasure-wrapper";
    }
}
var IfcHeatingValueMeasure = /** @class */ (function () {
    function IfcHeatingValueMeasure() {
    }
    return IfcHeatingValueMeasure;
}());
exports.IfcHeatingValueMeasure = IfcHeatingValueMeasure;
-wrapper;
{
    constructor(props ?  : IfcHeatingValueMeasure - wrapper);
    {
        this["@class"] = ".IfcHeatingValueMeasure-wrapper";
    }
}
var IfcIdentifier = /** @class */ (function () {
    function IfcIdentifier() {
    }
    return IfcIdentifier;
}());
exports.IfcIdentifier = IfcIdentifier;
-wrapper;
{
    constructor(props ?  : IfcIdentifier - wrapper);
    {
        this["@class"] = ".IfcIdentifier-wrapper";
    }
}
var IfcIlluminanceMeasure = /** @class */ (function () {
    function IfcIlluminanceMeasure() {
    }
    return IfcIlluminanceMeasure;
}());
exports.IfcIlluminanceMeasure = IfcIlluminanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcIlluminanceMeasure - wrapper);
    {
        this["@class"] = ".IfcIlluminanceMeasure-wrapper";
    }
}
var IfcInductanceMeasure = /** @class */ (function () {
    function IfcInductanceMeasure() {
    }
    return IfcInductanceMeasure;
}());
exports.IfcInductanceMeasure = IfcInductanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcInductanceMeasure - wrapper);
    {
        this["@class"] = ".IfcInductanceMeasure-wrapper";
    }
}
var IfcInteger = /** @class */ (function () {
    function IfcInteger() {
    }
    return IfcInteger;
}());
exports.IfcInteger = IfcInteger;
-wrapper;
{
    constructor(props ?  : IfcInteger - wrapper);
    {
        this["@class"] = ".IfcInteger-wrapper";
    }
}
var IfcIntegerCountRateMeasure = /** @class */ (function () {
    function IfcIntegerCountRateMeasure() {
    }
    return IfcIntegerCountRateMeasure;
}());
exports.IfcIntegerCountRateMeasure = IfcIntegerCountRateMeasure;
-wrapper;
{
    constructor(props ?  : IfcIntegerCountRateMeasure - wrapper);
    {
        this["@class"] = ".IfcIntegerCountRateMeasure-wrapper";
    }
}
var IfcIonConcentrationMeasure = /** @class */ (function () {
    function IfcIonConcentrationMeasure() {
    }
    return IfcIonConcentrationMeasure;
}());
exports.IfcIonConcentrationMeasure = IfcIonConcentrationMeasure;
-wrapper;
{
    constructor(props ?  : IfcIonConcentrationMeasure - wrapper);
    {
        this["@class"] = ".IfcIonConcentrationMeasure-wrapper";
    }
}
var IfcIsothermalMoistureCapacityMeasure = /** @class */ (function () {
    function IfcIsothermalMoistureCapacityMeasure() {
    }
    return IfcIsothermalMoistureCapacityMeasure;
}());
exports.IfcIsothermalMoistureCapacityMeasure = IfcIsothermalMoistureCapacityMeasure;
-wrapper;
{
    constructor(props ?  : IfcIsothermalMoistureCapacityMeasure - wrapper);
    {
        this["@class"] = ".IfcIsothermalMoistureCapacityMeasure-wrapper";
    }
}
var IfcKinematicViscosityMeasure = /** @class */ (function () {
    function IfcKinematicViscosityMeasure() {
    }
    return IfcKinematicViscosityMeasure;
}());
exports.IfcKinematicViscosityMeasure = IfcKinematicViscosityMeasure;
-wrapper;
{
    constructor(props ?  : IfcKinematicViscosityMeasure - wrapper);
    {
        this["@class"] = ".IfcKinematicViscosityMeasure-wrapper";
    }
}
var IfcLabel = /** @class */ (function () {
    function IfcLabel() {
    }
    return IfcLabel;
}());
exports.IfcLabel = IfcLabel;
-wrapper;
{
    constructor(props ?  : IfcLabel - wrapper);
    {
        this["@class"] = ".IfcLabel-wrapper";
    }
}
var IfcLengthMeasure = /** @class */ (function () {
    function IfcLengthMeasure() {
    }
    return IfcLengthMeasure;
}());
exports.IfcLengthMeasure = IfcLengthMeasure;
-wrapper;
{
    constructor(props ?  : IfcLengthMeasure - wrapper);
    {
        this["@class"] = ".IfcLengthMeasure-wrapper";
    }
}
var IfcLineIndex = /** @class */ (function () {
    function IfcLineIndex() {
    }
    return IfcLineIndex;
}());
exports.IfcLineIndex = IfcLineIndex;
-wrapper;
{
    constructor(props ?  : IfcLineIndex - wrapper);
    {
        this["@class"] = ".IfcLineIndex-wrapper";
    }
}
var IfcLinearForceMeasure = /** @class */ (function () {
    function IfcLinearForceMeasure() {
    }
    return IfcLinearForceMeasure;
}());
exports.IfcLinearForceMeasure = IfcLinearForceMeasure;
-wrapper;
{
    constructor(props ?  : IfcLinearForceMeasure - wrapper);
    {
        this["@class"] = ".IfcLinearForceMeasure-wrapper";
    }
}
var IfcLinearMomentMeasure = /** @class */ (function () {
    function IfcLinearMomentMeasure() {
    }
    return IfcLinearMomentMeasure;
}());
exports.IfcLinearMomentMeasure = IfcLinearMomentMeasure;
-wrapper;
{
    constructor(props ?  : IfcLinearMomentMeasure - wrapper);
    {
        this["@class"] = ".IfcLinearMomentMeasure-wrapper";
    }
}
var IfcLinearStiffnessMeasure = /** @class */ (function () {
    function IfcLinearStiffnessMeasure() {
    }
    return IfcLinearStiffnessMeasure;
}());
exports.IfcLinearStiffnessMeasure = IfcLinearStiffnessMeasure;
-wrapper;
{
    constructor(props ?  : IfcLinearStiffnessMeasure - wrapper);
    {
        this["@class"] = ".IfcLinearStiffnessMeasure-wrapper";
    }
}
var IfcLinearVelocityMeasure = /** @class */ (function () {
    function IfcLinearVelocityMeasure() {
    }
    return IfcLinearVelocityMeasure;
}());
exports.IfcLinearVelocityMeasure = IfcLinearVelocityMeasure;
-wrapper;
{
    constructor(props ?  : IfcLinearVelocityMeasure - wrapper);
    {
        this["@class"] = ".IfcLinearVelocityMeasure-wrapper";
    }
}
var IfcLogical = /** @class */ (function () {
    function IfcLogical() {
    }
    return IfcLogical;
}());
exports.IfcLogical = IfcLogical;
-wrapper;
{
    constructor(props ?  : IfcLogical - wrapper);
    {
        this["@class"] = ".IfcLogical-wrapper";
    }
}
var IfcLuminousFluxMeasure = /** @class */ (function () {
    function IfcLuminousFluxMeasure() {
    }
    return IfcLuminousFluxMeasure;
}());
exports.IfcLuminousFluxMeasure = IfcLuminousFluxMeasure;
-wrapper;
{
    constructor(props ?  : IfcLuminousFluxMeasure - wrapper);
    {
        this["@class"] = ".IfcLuminousFluxMeasure-wrapper";
    }
}
var IfcLuminousIntensityDistributionMeasure = /** @class */ (function () {
    function IfcLuminousIntensityDistributionMeasure() {
    }
    return IfcLuminousIntensityDistributionMeasure;
}());
exports.IfcLuminousIntensityDistributionMeasure = IfcLuminousIntensityDistributionMeasure;
-wrapper;
{
    constructor(props ?  : IfcLuminousIntensityDistributionMeasure - wrapper);
    {
        this["@class"] = ".IfcLuminousIntensityDistributionMeasure-wrapper";
    }
}
var IfcLuminousIntensityMeasure = /** @class */ (function () {
    function IfcLuminousIntensityMeasure() {
    }
    return IfcLuminousIntensityMeasure;
}());
exports.IfcLuminousIntensityMeasure = IfcLuminousIntensityMeasure;
-wrapper;
{
    constructor(props ?  : IfcLuminousIntensityMeasure - wrapper);
    {
        this["@class"] = ".IfcLuminousIntensityMeasure-wrapper";
    }
}
var IfcMagneticFluxDensityMeasure = /** @class */ (function () {
    function IfcMagneticFluxDensityMeasure() {
    }
    return IfcMagneticFluxDensityMeasure;
}());
exports.IfcMagneticFluxDensityMeasure = IfcMagneticFluxDensityMeasure;
-wrapper;
{
    constructor(props ?  : IfcMagneticFluxDensityMeasure - wrapper);
    {
        this["@class"] = ".IfcMagneticFluxDensityMeasure-wrapper";
    }
}
var IfcMagneticFluxMeasure = /** @class */ (function () {
    function IfcMagneticFluxMeasure() {
    }
    return IfcMagneticFluxMeasure;
}());
exports.IfcMagneticFluxMeasure = IfcMagneticFluxMeasure;
-wrapper;
{
    constructor(props ?  : IfcMagneticFluxMeasure - wrapper);
    {
        this["@class"] = ".IfcMagneticFluxMeasure-wrapper";
    }
}
var IfcMassDensityMeasure = /** @class */ (function () {
    function IfcMassDensityMeasure() {
    }
    return IfcMassDensityMeasure;
}());
exports.IfcMassDensityMeasure = IfcMassDensityMeasure;
-wrapper;
{
    constructor(props ?  : IfcMassDensityMeasure - wrapper);
    {
        this["@class"] = ".IfcMassDensityMeasure-wrapper";
    }
}
var IfcMassFlowRateMeasure = /** @class */ (function () {
    function IfcMassFlowRateMeasure() {
    }
    return IfcMassFlowRateMeasure;
}());
exports.IfcMassFlowRateMeasure = IfcMassFlowRateMeasure;
-wrapper;
{
    constructor(props ?  : IfcMassFlowRateMeasure - wrapper);
    {
        this["@class"] = ".IfcMassFlowRateMeasure-wrapper";
    }
}
var IfcMassMeasure = /** @class */ (function () {
    function IfcMassMeasure() {
    }
    return IfcMassMeasure;
}());
exports.IfcMassMeasure = IfcMassMeasure;
-wrapper;
{
    constructor(props ?  : IfcMassMeasure - wrapper);
    {
        this["@class"] = ".IfcMassMeasure-wrapper";
    }
}
var IfcMassPerLengthMeasure = /** @class */ (function () {
    function IfcMassPerLengthMeasure() {
    }
    return IfcMassPerLengthMeasure;
}());
exports.IfcMassPerLengthMeasure = IfcMassPerLengthMeasure;
-wrapper;
{
    constructor(props ?  : IfcMassPerLengthMeasure - wrapper);
    {
        this["@class"] = ".IfcMassPerLengthMeasure-wrapper";
    }
}
var IfcModulusOfElasticityMeasure = /** @class */ (function () {
    function IfcModulusOfElasticityMeasure() {
    }
    return IfcModulusOfElasticityMeasure;
}());
exports.IfcModulusOfElasticityMeasure = IfcModulusOfElasticityMeasure;
-wrapper;
{
    constructor(props ?  : IfcModulusOfElasticityMeasure - wrapper);
    {
        this["@class"] = ".IfcModulusOfElasticityMeasure-wrapper";
    }
}
var IfcModulusOfLinearSubgradeReactionMeasure = /** @class */ (function () {
    function IfcModulusOfLinearSubgradeReactionMeasure() {
    }
    return IfcModulusOfLinearSubgradeReactionMeasure;
}());
exports.IfcModulusOfLinearSubgradeReactionMeasure = IfcModulusOfLinearSubgradeReactionMeasure;
-wrapper;
{
    constructor(props ?  : IfcModulusOfLinearSubgradeReactionMeasure - wrapper);
    {
        this["@class"] = ".IfcModulusOfLinearSubgradeReactionMeasure-wrapper";
    }
}
var IfcModulusOfRotationalSubgradeReactionMeasure = /** @class */ (function () {
    function IfcModulusOfRotationalSubgradeReactionMeasure() {
    }
    return IfcModulusOfRotationalSubgradeReactionMeasure;
}());
exports.IfcModulusOfRotationalSubgradeReactionMeasure = IfcModulusOfRotationalSubgradeReactionMeasure;
-wrapper;
{
    constructor(props ?  : IfcModulusOfRotationalSubgradeReactionMeasure - wrapper);
    {
        this["@class"] = ".IfcModulusOfRotationalSubgradeReactionMeasure-wrapper";
    }
}
var IfcModulusOfSubgradeReactionMeasure = /** @class */ (function () {
    function IfcModulusOfSubgradeReactionMeasure() {
    }
    return IfcModulusOfSubgradeReactionMeasure;
}());
exports.IfcModulusOfSubgradeReactionMeasure = IfcModulusOfSubgradeReactionMeasure;
-wrapper;
{
    constructor(props ?  : IfcModulusOfSubgradeReactionMeasure - wrapper);
    {
        this["@class"] = ".IfcModulusOfSubgradeReactionMeasure-wrapper";
    }
}
var IfcMoistureDiffusivityMeasure = /** @class */ (function () {
    function IfcMoistureDiffusivityMeasure() {
    }
    return IfcMoistureDiffusivityMeasure;
}());
exports.IfcMoistureDiffusivityMeasure = IfcMoistureDiffusivityMeasure;
-wrapper;
{
    constructor(props ?  : IfcMoistureDiffusivityMeasure - wrapper);
    {
        this["@class"] = ".IfcMoistureDiffusivityMeasure-wrapper";
    }
}
var IfcMolecularWeightMeasure = /** @class */ (function () {
    function IfcMolecularWeightMeasure() {
    }
    return IfcMolecularWeightMeasure;
}());
exports.IfcMolecularWeightMeasure = IfcMolecularWeightMeasure;
-wrapper;
{
    constructor(props ?  : IfcMolecularWeightMeasure - wrapper);
    {
        this["@class"] = ".IfcMolecularWeightMeasure-wrapper";
    }
}
var IfcMomentOfInertiaMeasure = /** @class */ (function () {
    function IfcMomentOfInertiaMeasure() {
    }
    return IfcMomentOfInertiaMeasure;
}());
exports.IfcMomentOfInertiaMeasure = IfcMomentOfInertiaMeasure;
-wrapper;
{
    constructor(props ?  : IfcMomentOfInertiaMeasure - wrapper);
    {
        this["@class"] = ".IfcMomentOfInertiaMeasure-wrapper";
    }
}
var IfcMonetaryMeasure = /** @class */ (function () {
    function IfcMonetaryMeasure() {
    }
    return IfcMonetaryMeasure;
}());
exports.IfcMonetaryMeasure = IfcMonetaryMeasure;
-wrapper;
{
    constructor(props ?  : IfcMonetaryMeasure - wrapper);
    {
        this["@class"] = ".IfcMonetaryMeasure-wrapper";
    }
}
var IfcNonNegativeLengthMeasure = /** @class */ (function () {
    function IfcNonNegativeLengthMeasure() {
    }
    return IfcNonNegativeLengthMeasure;
}());
exports.IfcNonNegativeLengthMeasure = IfcNonNegativeLengthMeasure;
-wrapper;
{
    constructor(props ?  : IfcNonNegativeLengthMeasure - wrapper);
    {
        this["@class"] = ".IfcNonNegativeLengthMeasure-wrapper";
    }
}
var IfcNormalisedRatioMeasure = /** @class */ (function () {
    function IfcNormalisedRatioMeasure() {
    }
    return IfcNormalisedRatioMeasure;
}());
exports.IfcNormalisedRatioMeasure = IfcNormalisedRatioMeasure;
-wrapper;
{
    constructor(props ?  : IfcNormalisedRatioMeasure - wrapper);
    {
        this["@class"] = ".IfcNormalisedRatioMeasure-wrapper";
    }
}
var IfcNullStyle = /** @class */ (function () {
    function IfcNullStyle() {
    }
    return IfcNullStyle;
}());
exports.IfcNullStyle = IfcNullStyle;
-wrapper;
{
    constructor(props ?  : IfcNullStyle - wrapper);
    {
        this["@class"] = ".IfcNullStyle-wrapper";
    }
}
var IfcNumericMeasure = /** @class */ (function () {
    function IfcNumericMeasure() {
    }
    return IfcNumericMeasure;
}());
exports.IfcNumericMeasure = IfcNumericMeasure;
-wrapper;
{
    constructor(props ?  : IfcNumericMeasure - wrapper);
    {
        this["@class"] = ".IfcNumericMeasure-wrapper";
    }
}
var IfcPHMeasure = /** @class */ (function () {
    function IfcPHMeasure() {
    }
    return IfcPHMeasure;
}());
exports.IfcPHMeasure = IfcPHMeasure;
-wrapper;
{
    constructor(props ?  : IfcPHMeasure - wrapper);
    {
        this["@class"] = ".IfcPHMeasure-wrapper";
    }
}
var IfcParameterValue = /** @class */ (function () {
    function IfcParameterValue() {
    }
    return IfcParameterValue;
}());
exports.IfcParameterValue = IfcParameterValue;
-wrapper;
{
    constructor(props ?  : IfcParameterValue - wrapper);
    {
        this["@class"] = ".IfcParameterValue-wrapper";
    }
}
var IfcPlanarForceMeasure = /** @class */ (function () {
    function IfcPlanarForceMeasure() {
    }
    return IfcPlanarForceMeasure;
}());
exports.IfcPlanarForceMeasure = IfcPlanarForceMeasure;
-wrapper;
{
    constructor(props ?  : IfcPlanarForceMeasure - wrapper);
    {
        this["@class"] = ".IfcPlanarForceMeasure-wrapper";
    }
}
var IfcPlaneAngleMeasure = /** @class */ (function () {
    function IfcPlaneAngleMeasure() {
    }
    return IfcPlaneAngleMeasure;
}());
exports.IfcPlaneAngleMeasure = IfcPlaneAngleMeasure;
-wrapper;
{
    constructor(props ?  : IfcPlaneAngleMeasure - wrapper);
    {
        this["@class"] = ".IfcPlaneAngleMeasure-wrapper";
    }
}
var IfcPositiveInteger = /** @class */ (function () {
    function IfcPositiveInteger() {
    }
    return IfcPositiveInteger;
}());
exports.IfcPositiveInteger = IfcPositiveInteger;
-wrapper;
{
    constructor(props ?  : IfcPositiveInteger - wrapper);
    {
        this["@class"] = ".IfcPositiveInteger-wrapper";
    }
}
var IfcPositiveLengthMeasure = /** @class */ (function () {
    function IfcPositiveLengthMeasure() {
    }
    return IfcPositiveLengthMeasure;
}());
exports.IfcPositiveLengthMeasure = IfcPositiveLengthMeasure;
-wrapper;
{
    constructor(props ?  : IfcPositiveLengthMeasure - wrapper);
    {
        this["@class"] = ".IfcPositiveLengthMeasure-wrapper";
    }
}
var IfcPositivePlaneAngleMeasure = /** @class */ (function () {
    function IfcPositivePlaneAngleMeasure() {
    }
    return IfcPositivePlaneAngleMeasure;
}());
exports.IfcPositivePlaneAngleMeasure = IfcPositivePlaneAngleMeasure;
-wrapper;
{
    constructor(props ?  : IfcPositivePlaneAngleMeasure - wrapper);
    {
        this["@class"] = ".IfcPositivePlaneAngleMeasure-wrapper";
    }
}
var IfcPositiveRatioMeasure = /** @class */ (function () {
    function IfcPositiveRatioMeasure() {
    }
    return IfcPositiveRatioMeasure;
}());
exports.IfcPositiveRatioMeasure = IfcPositiveRatioMeasure;
-wrapper;
{
    constructor(props ?  : IfcPositiveRatioMeasure - wrapper);
    {
        this["@class"] = ".IfcPositiveRatioMeasure-wrapper";
    }
}
var IfcPowerMeasure = /** @class */ (function () {
    function IfcPowerMeasure() {
    }
    return IfcPowerMeasure;
}());
exports.IfcPowerMeasure = IfcPowerMeasure;
-wrapper;
{
    constructor(props ?  : IfcPowerMeasure - wrapper);
    {
        this["@class"] = ".IfcPowerMeasure-wrapper";
    }
}
var IfcPressureMeasure = /** @class */ (function () {
    function IfcPressureMeasure() {
    }
    return IfcPressureMeasure;
}());
exports.IfcPressureMeasure = IfcPressureMeasure;
-wrapper;
{
    constructor(props ?  : IfcPressureMeasure - wrapper);
    {
        this["@class"] = ".IfcPressureMeasure-wrapper";
    }
}
var IfcPropertySetDefinitionSet = /** @class */ (function () {
    function IfcPropertySetDefinitionSet() {
    }
    return IfcPropertySetDefinitionSet;
}());
exports.IfcPropertySetDefinitionSet = IfcPropertySetDefinitionSet;
-wrapper;
{
    constructor(props ?  : IfcPropertySetDefinitionSet - wrapper);
    {
        this["@class"] = ".IfcPropertySetDefinitionSet-wrapper";
    }
}
var IfcRadioActivityMeasure = /** @class */ (function () {
    function IfcRadioActivityMeasure() {
    }
    return IfcRadioActivityMeasure;
}());
exports.IfcRadioActivityMeasure = IfcRadioActivityMeasure;
-wrapper;
{
    constructor(props ?  : IfcRadioActivityMeasure - wrapper);
    {
        this["@class"] = ".IfcRadioActivityMeasure-wrapper";
    }
}
var IfcRatioMeasure = /** @class */ (function () {
    function IfcRatioMeasure() {
    }
    return IfcRatioMeasure;
}());
exports.IfcRatioMeasure = IfcRatioMeasure;
-wrapper;
{
    constructor(props ?  : IfcRatioMeasure - wrapper);
    {
        this["@class"] = ".IfcRatioMeasure-wrapper";
    }
}
var IfcReal = /** @class */ (function () {
    function IfcReal() {
    }
    return IfcReal;
}());
exports.IfcReal = IfcReal;
-wrapper;
{
    constructor(props ?  : IfcReal - wrapper);
    {
        this["@class"] = ".IfcReal-wrapper";
    }
}
var IfcRotationalFrequencyMeasure = /** @class */ (function () {
    function IfcRotationalFrequencyMeasure() {
    }
    return IfcRotationalFrequencyMeasure;
}());
exports.IfcRotationalFrequencyMeasure = IfcRotationalFrequencyMeasure;
-wrapper;
{
    constructor(props ?  : IfcRotationalFrequencyMeasure - wrapper);
    {
        this["@class"] = ".IfcRotationalFrequencyMeasure-wrapper";
    }
}
var IfcRotationalMassMeasure = /** @class */ (function () {
    function IfcRotationalMassMeasure() {
    }
    return IfcRotationalMassMeasure;
}());
exports.IfcRotationalMassMeasure = IfcRotationalMassMeasure;
-wrapper;
{
    constructor(props ?  : IfcRotationalMassMeasure - wrapper);
    {
        this["@class"] = ".IfcRotationalMassMeasure-wrapper";
    }
}
var IfcRotationalStiffnessMeasure = /** @class */ (function () {
    function IfcRotationalStiffnessMeasure() {
    }
    return IfcRotationalStiffnessMeasure;
}());
exports.IfcRotationalStiffnessMeasure = IfcRotationalStiffnessMeasure;
-wrapper;
{
    constructor(props ?  : IfcRotationalStiffnessMeasure - wrapper);
    {
        this["@class"] = ".IfcRotationalStiffnessMeasure-wrapper";
    }
}
var IfcSectionModulusMeasure = /** @class */ (function () {
    function IfcSectionModulusMeasure() {
    }
    return IfcSectionModulusMeasure;
}());
exports.IfcSectionModulusMeasure = IfcSectionModulusMeasure;
-wrapper;
{
    constructor(props ?  : IfcSectionModulusMeasure - wrapper);
    {
        this["@class"] = ".IfcSectionModulusMeasure-wrapper";
    }
}
var IfcSectionalAreaIntegralMeasure = /** @class */ (function () {
    function IfcSectionalAreaIntegralMeasure() {
    }
    return IfcSectionalAreaIntegralMeasure;
}());
exports.IfcSectionalAreaIntegralMeasure = IfcSectionalAreaIntegralMeasure;
-wrapper;
{
    constructor(props ?  : IfcSectionalAreaIntegralMeasure - wrapper);
    {
        this["@class"] = ".IfcSectionalAreaIntegralMeasure-wrapper";
    }
}
var IfcShearModulusMeasure = /** @class */ (function () {
    function IfcShearModulusMeasure() {
    }
    return IfcShearModulusMeasure;
}());
exports.IfcShearModulusMeasure = IfcShearModulusMeasure;
-wrapper;
{
    constructor(props ?  : IfcShearModulusMeasure - wrapper);
    {
        this["@class"] = ".IfcShearModulusMeasure-wrapper";
    }
}
var IfcSolidAngleMeasure = /** @class */ (function () {
    function IfcSolidAngleMeasure() {
    }
    return IfcSolidAngleMeasure;
}());
exports.IfcSolidAngleMeasure = IfcSolidAngleMeasure;
-wrapper;
{
    constructor(props ?  : IfcSolidAngleMeasure - wrapper);
    {
        this["@class"] = ".IfcSolidAngleMeasure-wrapper";
    }
}
var IfcSoundPowerLevelMeasure = /** @class */ (function () {
    function IfcSoundPowerLevelMeasure() {
    }
    return IfcSoundPowerLevelMeasure;
}());
exports.IfcSoundPowerLevelMeasure = IfcSoundPowerLevelMeasure;
-wrapper;
{
    constructor(props ?  : IfcSoundPowerLevelMeasure - wrapper);
    {
        this["@class"] = ".IfcSoundPowerLevelMeasure-wrapper";
    }
}
var IfcSoundPowerMeasure = /** @class */ (function () {
    function IfcSoundPowerMeasure() {
    }
    return IfcSoundPowerMeasure;
}());
exports.IfcSoundPowerMeasure = IfcSoundPowerMeasure;
-wrapper;
{
    constructor(props ?  : IfcSoundPowerMeasure - wrapper);
    {
        this["@class"] = ".IfcSoundPowerMeasure-wrapper";
    }
}
var IfcSoundPressureLevelMeasure = /** @class */ (function () {
    function IfcSoundPressureLevelMeasure() {
    }
    return IfcSoundPressureLevelMeasure;
}());
exports.IfcSoundPressureLevelMeasure = IfcSoundPressureLevelMeasure;
-wrapper;
{
    constructor(props ?  : IfcSoundPressureLevelMeasure - wrapper);
    {
        this["@class"] = ".IfcSoundPressureLevelMeasure-wrapper";
    }
}
var IfcSoundPressureMeasure = /** @class */ (function () {
    function IfcSoundPressureMeasure() {
    }
    return IfcSoundPressureMeasure;
}());
exports.IfcSoundPressureMeasure = IfcSoundPressureMeasure;
-wrapper;
{
    constructor(props ?  : IfcSoundPressureMeasure - wrapper);
    {
        this["@class"] = ".IfcSoundPressureMeasure-wrapper";
    }
}
var IfcSpecificHeatCapacityMeasure = /** @class */ (function () {
    function IfcSpecificHeatCapacityMeasure() {
    }
    return IfcSpecificHeatCapacityMeasure;
}());
exports.IfcSpecificHeatCapacityMeasure = IfcSpecificHeatCapacityMeasure;
-wrapper;
{
    constructor(props ?  : IfcSpecificHeatCapacityMeasure - wrapper);
    {
        this["@class"] = ".IfcSpecificHeatCapacityMeasure-wrapper";
    }
}
var IfcSpecularExponent = /** @class */ (function () {
    function IfcSpecularExponent() {
    }
    return IfcSpecularExponent;
}());
exports.IfcSpecularExponent = IfcSpecularExponent;
-wrapper;
{
    constructor(props ?  : IfcSpecularExponent - wrapper);
    {
        this["@class"] = ".IfcSpecularExponent-wrapper";
    }
}
var IfcSpecularRoughness = /** @class */ (function () {
    function IfcSpecularRoughness() {
    }
    return IfcSpecularRoughness;
}());
exports.IfcSpecularRoughness = IfcSpecularRoughness;
-wrapper;
{
    constructor(props ?  : IfcSpecularRoughness - wrapper);
    {
        this["@class"] = ".IfcSpecularRoughness-wrapper";
    }
}
var IfcTemperatureGradientMeasure = /** @class */ (function () {
    function IfcTemperatureGradientMeasure() {
    }
    return IfcTemperatureGradientMeasure;
}());
exports.IfcTemperatureGradientMeasure = IfcTemperatureGradientMeasure;
-wrapper;
{
    constructor(props ?  : IfcTemperatureGradientMeasure - wrapper);
    {
        this["@class"] = ".IfcTemperatureGradientMeasure-wrapper";
    }
}
var IfcTemperatureRateOfChangeMeasure = /** @class */ (function () {
    function IfcTemperatureRateOfChangeMeasure() {
    }
    return IfcTemperatureRateOfChangeMeasure;
}());
exports.IfcTemperatureRateOfChangeMeasure = IfcTemperatureRateOfChangeMeasure;
-wrapper;
{
    constructor(props ?  : IfcTemperatureRateOfChangeMeasure - wrapper);
    {
        this["@class"] = ".IfcTemperatureRateOfChangeMeasure-wrapper";
    }
}
var IfcText = /** @class */ (function () {
    function IfcText() {
    }
    return IfcText;
}());
exports.IfcText = IfcText;
-wrapper;
{
    constructor(props ?  : IfcText - wrapper);
    {
        this["@class"] = ".IfcText-wrapper";
    }
}
var IfcTextFontName = /** @class */ (function () {
    function IfcTextFontName() {
    }
    return IfcTextFontName;
}());
exports.IfcTextFontName = IfcTextFontName;
-wrapper;
{
    constructor(props ?  : IfcTextFontName - wrapper);
    {
        this["@class"] = ".IfcTextFontName-wrapper";
    }
}
var IfcThermalAdmittanceMeasure = /** @class */ (function () {
    function IfcThermalAdmittanceMeasure() {
    }
    return IfcThermalAdmittanceMeasure;
}());
exports.IfcThermalAdmittanceMeasure = IfcThermalAdmittanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermalAdmittanceMeasure - wrapper);
    {
        this["@class"] = ".IfcThermalAdmittanceMeasure-wrapper";
    }
}
var IfcThermalConductivityMeasure = /** @class */ (function () {
    function IfcThermalConductivityMeasure() {
    }
    return IfcThermalConductivityMeasure;
}());
exports.IfcThermalConductivityMeasure = IfcThermalConductivityMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermalConductivityMeasure - wrapper);
    {
        this["@class"] = ".IfcThermalConductivityMeasure-wrapper";
    }
}
var IfcThermalExpansionCoefficientMeasure = /** @class */ (function () {
    function IfcThermalExpansionCoefficientMeasure() {
    }
    return IfcThermalExpansionCoefficientMeasure;
}());
exports.IfcThermalExpansionCoefficientMeasure = IfcThermalExpansionCoefficientMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermalExpansionCoefficientMeasure - wrapper);
    {
        this["@class"] = ".IfcThermalExpansionCoefficientMeasure-wrapper";
    }
}
var IfcThermalResistanceMeasure = /** @class */ (function () {
    function IfcThermalResistanceMeasure() {
    }
    return IfcThermalResistanceMeasure;
}());
exports.IfcThermalResistanceMeasure = IfcThermalResistanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermalResistanceMeasure - wrapper);
    {
        this["@class"] = ".IfcThermalResistanceMeasure-wrapper";
    }
}
var IfcThermalTransmittanceMeasure = /** @class */ (function () {
    function IfcThermalTransmittanceMeasure() {
    }
    return IfcThermalTransmittanceMeasure;
}());
exports.IfcThermalTransmittanceMeasure = IfcThermalTransmittanceMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermalTransmittanceMeasure - wrapper);
    {
        this["@class"] = ".IfcThermalTransmittanceMeasure-wrapper";
    }
}
var IfcThermodynamicTemperatureMeasure = /** @class */ (function () {
    function IfcThermodynamicTemperatureMeasure() {
    }
    return IfcThermodynamicTemperatureMeasure;
}());
exports.IfcThermodynamicTemperatureMeasure = IfcThermodynamicTemperatureMeasure;
-wrapper;
{
    constructor(props ?  : IfcThermodynamicTemperatureMeasure - wrapper);
    {
        this["@class"] = ".IfcThermodynamicTemperatureMeasure-wrapper";
    }
}
var IfcTime = /** @class */ (function () {
    function IfcTime() {
    }
    return IfcTime;
}());
exports.IfcTime = IfcTime;
-wrapper;
{
    constructor(props ?  : IfcTime - wrapper);
    {
        this["@class"] = ".IfcTime-wrapper";
    }
}
var IfcTimeMeasure = /** @class */ (function () {
    function IfcTimeMeasure() {
    }
    return IfcTimeMeasure;
}());
exports.IfcTimeMeasure = IfcTimeMeasure;
-wrapper;
{
    constructor(props ?  : IfcTimeMeasure - wrapper);
    {
        this["@class"] = ".IfcTimeMeasure-wrapper";
    }
}
var IfcTimeStamp = /** @class */ (function () {
    function IfcTimeStamp() {
    }
    return IfcTimeStamp;
}());
exports.IfcTimeStamp = IfcTimeStamp;
-wrapper;
{
    constructor(props ?  : IfcTimeStamp - wrapper);
    {
        this["@class"] = ".IfcTimeStamp-wrapper";
    }
}
var IfcTorqueMeasure = /** @class */ (function () {
    function IfcTorqueMeasure() {
    }
    return IfcTorqueMeasure;
}());
exports.IfcTorqueMeasure = IfcTorqueMeasure;
-wrapper;
{
    constructor(props ?  : IfcTorqueMeasure - wrapper);
    {
        this["@class"] = ".IfcTorqueMeasure-wrapper";
    }
}
var IfcVaporPermeabilityMeasure = /** @class */ (function () {
    function IfcVaporPermeabilityMeasure() {
    }
    return IfcVaporPermeabilityMeasure;
}());
exports.IfcVaporPermeabilityMeasure = IfcVaporPermeabilityMeasure;
-wrapper;
{
    constructor(props ?  : IfcVaporPermeabilityMeasure - wrapper);
    {
        this["@class"] = ".IfcVaporPermeabilityMeasure-wrapper";
    }
}
var IfcVolumeMeasure = /** @class */ (function () {
    function IfcVolumeMeasure() {
    }
    return IfcVolumeMeasure;
}());
exports.IfcVolumeMeasure = IfcVolumeMeasure;
-wrapper;
{
    constructor(props ?  : IfcVolumeMeasure - wrapper);
    {
        this["@class"] = ".IfcVolumeMeasure-wrapper";
    }
}
var IfcVolumetricFlowRateMeasure = /** @class */ (function () {
    function IfcVolumetricFlowRateMeasure() {
    }
    return IfcVolumetricFlowRateMeasure;
}());
exports.IfcVolumetricFlowRateMeasure = IfcVolumetricFlowRateMeasure;
-wrapper;
{
    constructor(props ?  : IfcVolumetricFlowRateMeasure - wrapper);
    {
        this["@class"] = ".IfcVolumetricFlowRateMeasure-wrapper";
    }
}
var IfcWarpingConstantMeasure = /** @class */ (function () {
    function IfcWarpingConstantMeasure() {
    }
    return IfcWarpingConstantMeasure;
}());
exports.IfcWarpingConstantMeasure = IfcWarpingConstantMeasure;
-wrapper;
{
    constructor(props ?  : IfcWarpingConstantMeasure - wrapper);
    {
        this["@class"] = ".IfcWarpingConstantMeasure-wrapper";
    }
}
var IfcWarpingMomentMeasure = /** @class */ (function () {
    function IfcWarpingMomentMeasure() {
    }
    return IfcWarpingMomentMeasure;
}());
exports.IfcWarpingMomentMeasure = IfcWarpingMomentMeasure;
-wrapper;
{
    constructor(props ?  : IfcWarpingMomentMeasure - wrapper);
    {
        this["@class"] = ".IfcWarpingMomentMeasure-wrapper";
    }
}
var IfcAdvancedBrepWithVoids = /** @class */ (function (_super) {
    __extends(IfcAdvancedBrepWithVoids, _super);
    function IfcAdvancedBrepWithVoids(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAdvancedBrepWithVoids";
        if (props) {
            _this.Voids = (_a = props.Voids) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcClosedShell(o); });
        }
        return _this;
    }
    return IfcAdvancedBrepWithVoids;
}(IfcAdvancedBrep));
exports.IfcAdvancedBrepWithVoids = IfcAdvancedBrepWithVoids;
var IfcAnnotationFillArea = /** @class */ (function (_super) {
    __extends(IfcAnnotationFillArea, _super);
    function IfcAnnotationFillArea(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAnnotationFillArea";
        if (props) {
            _this.OuterBoundary = (props.OuterBoundary) ? new IfcCurve(props.OuterBoundary) : undefined;
            _this.InnerBoundaries = (_a = props.InnerBoundaries) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCurve(o); });
        }
        return _this;
    }
    return IfcAnnotationFillArea;
}(IfcGeometricRepresentationItem));
exports.IfcAnnotationFillArea = IfcAnnotationFillArea;
var IfcApplication = /** @class */ (function (_super) {
    __extends(IfcApplication, _super);
    function IfcApplication(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcApplication";
        if (props) {
            _this.ApplicationDeveloper = (props.ApplicationDeveloper) ? new IfcOrganization(props.ApplicationDeveloper) : undefined;
        }
        return _this;
    }
    return IfcApplication;
}(Entity));
exports.IfcApplication = IfcApplication;
var IfcAppliedValue = /** @class */ (function (_super) {
    __extends(IfcAppliedValue, _super);
    function IfcAppliedValue(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAppliedValue";
        if (props) {
            _this.UnitBasis = (props.UnitBasis) ? new IfcMeasureWithUnit(props.UnitBasis) : undefined;
            _this.Components = (_a = props.Components) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcAppliedValue(o); });
        }
        return _this;
    }
    return IfcAppliedValue;
}(Entity));
exports.IfcAppliedValue = IfcAppliedValue;
var IfcApproval = /** @class */ (function (_super) {
    __extends(IfcApproval, _super);
    function IfcApproval(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcApproval";
        return _this;
    }
    return IfcApproval;
}(Entity));
exports.IfcApproval = IfcApproval;
var IfcApprovalRelationship = /** @class */ (function (_super) {
    __extends(IfcApprovalRelationship, _super);
    function IfcApprovalRelationship(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcApprovalRelationship";
        if (props) {
            _this.RelatingApproval = (props.RelatingApproval) ? new IfcApproval(props.RelatingApproval) : undefined;
            _this.RelatedApprovals = (_a = props.RelatedApprovals) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcApproval(o); });
        }
        return _this;
    }
    return IfcApprovalRelationship;
}(IfcResourceLevelRelationship));
exports.IfcApprovalRelationship = IfcApprovalRelationship;
var IfcAsset = /** @class */ (function (_super) {
    __extends(IfcAsset, _super);
    function IfcAsset(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAsset";
        if (props) {
            _this.OriginalValue = (props.OriginalValue) ? new IfcCostValue(props.OriginalValue) : undefined;
            _this.CurrentValue = (props.CurrentValue) ? new IfcCostValue(props.CurrentValue) : undefined;
            _this.TotalReplacementCost = (props.TotalReplacementCost) ? new IfcCostValue(props.TotalReplacementCost) : undefined;
            _this.ResponsiblePerson = (props.ResponsiblePerson) ? new IfcPerson(props.ResponsiblePerson) : undefined;
            _this.DepreciatedValue = (props.DepreciatedValue) ? new IfcCostValue(props.DepreciatedValue) : undefined;
        }
        return _this;
    }
    return IfcAsset;
}(IfcGroup));
exports.IfcAsset = IfcAsset;
var IfcBSplineCurve = /** @class */ (function (_super) {
    __extends(IfcBSplineCurve, _super);
    function IfcBSplineCurve(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBSplineCurve";
        if (props) {
            _this.ControlPointsList = (_a = props.ControlPointsList) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCartesianPoint(o); });
        }
        return _this;
    }
    return IfcBSplineCurve;
}(IfcBoundedCurve));
exports.IfcBSplineCurve = IfcBSplineCurve;
var IfcBSplineSurface = /** @class */ (function (_super) {
    __extends(IfcBSplineSurface, _super);
    function IfcBSplineSurface(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBSplineSurface";
        if (props) {
            _this.ControlPointsList = (_a = props.ControlPointsList) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCartesianPoint(o); });
        }
        return _this;
    }
    return IfcBSplineSurface;
}(IfcBoundedSurface));
exports.IfcBSplineSurface = IfcBSplineSurface;
var IfcBooleanResult = /** @class */ (function (_super) {
    __extends(IfcBooleanResult, _super);
    function IfcBooleanResult(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBooleanResult";
        return _this;
    }
    return IfcBooleanResult;
}(IfcGeometricRepresentationItem));
exports.IfcBooleanResult = IfcBooleanResult;
var IfcBoundaryEdgeCondition = /** @class */ (function (_super) {
    __extends(IfcBoundaryEdgeCondition, _super);
    function IfcBoundaryEdgeCondition(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoundaryEdgeCondition";
        return _this;
    }
    return IfcBoundaryEdgeCondition;
}(IfcBoundaryCondition));
exports.IfcBoundaryEdgeCondition = IfcBoundaryEdgeCondition;
var IfcBoundaryFaceCondition = /** @class */ (function (_super) {
    __extends(IfcBoundaryFaceCondition, _super);
    function IfcBoundaryFaceCondition(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoundaryFaceCondition";
        return _this;
    }
    return IfcBoundaryFaceCondition;
}(IfcBoundaryCondition));
exports.IfcBoundaryFaceCondition = IfcBoundaryFaceCondition;
var IfcBoundaryNodeCondition = /** @class */ (function (_super) {
    __extends(IfcBoundaryNodeCondition, _super);
    function IfcBoundaryNodeCondition(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoundaryNodeCondition";
        return _this;
    }
    return IfcBoundaryNodeCondition;
}(IfcBoundaryCondition));
exports.IfcBoundaryNodeCondition = IfcBoundaryNodeCondition;
var IfcBoundingBox = /** @class */ (function (_super) {
    __extends(IfcBoundingBox, _super);
    function IfcBoundingBox(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoundingBox";
        if (props) {
            _this.Corner = (props.Corner) ? new IfcCartesianPoint(props.Corner) : undefined;
        }
        return _this;
    }
    return IfcBoundingBox;
}(IfcGeometricRepresentationItem));
exports.IfcBoundingBox = IfcBoundingBox;
var IfcBuilding = /** @class */ (function (_super) {
    __extends(IfcBuilding, _super);
    function IfcBuilding(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBuilding";
        if (props) {
            _this.BuildingAddress = props.BuildingAddress;
        }
        return _this;
    }
    return IfcBuilding;
}(IfcSpatialStructureElement));
exports.IfcBuilding = IfcBuilding;
var IfcCartesianTransformationOperator = /** @class */ (function (_super) {
    __extends(IfcCartesianTransformationOperator, _super);
    function IfcCartesianTransformationOperator(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCartesianTransformationOperator";
        if (props) {
            _this.Axis1 = (props.Axis1) ? new IfcDirection(props.Axis1) : undefined;
            _this.Axis2 = (props.Axis2) ? new IfcDirection(props.Axis2) : undefined;
            _this.LocalOrigin = (props.LocalOrigin) ? new IfcCartesianPoint(props.LocalOrigin) : undefined;
        }
        return _this;
    }
    return IfcCartesianTransformationOperator;
}(IfcGeometricRepresentationItem));
exports.IfcCartesianTransformationOperator = IfcCartesianTransformationOperator;
var IfcClassification = /** @class */ (function (_super) {
    __extends(IfcClassification, _super);
    function IfcClassification() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcClassification;
}(IfcExternalInformation));
exports.IfcClassification = IfcClassification;
-wrapper[];
HasReferences ?  : IfcClassificationReference[];
constructor(props ?  : IfcClassification);
{
    _this = _super.call(this, props) || this;
    this["@class"] = ".IfcClassification";
    if (props) {
        this.ReferenceTokens = (_b = props.ReferenceTokens) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcIdentifier - wrapper(o); });
        this.HasReferences = (_c = props.HasReferences) === null || _c === void 0 ? void 0 : _c.map(function (o) { return new IfcClassificationReference(o); });
    }
}
var IfcClassificationReference = /** @class */ (function (_super) {
    __extends(IfcClassificationReference, _super);
    function IfcClassificationReference(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcClassificationReference";
        if (props) {
            _this.HasReferences = (_a = props.HasReferences) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcClassificationReference(o); });
        }
        return _this;
    }
    return IfcClassificationReference;
}(IfcExternalReference));
exports.IfcClassificationReference = IfcClassificationReference;
var IfcComplexProperty = /** @class */ (function (_super) {
    __extends(IfcComplexProperty, _super);
    function IfcComplexProperty(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcComplexProperty";
        if (props) {
            _this.HasProperties = (_a = props.HasProperties) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcComplexProperty;
}(IfcProperty));
exports.IfcComplexProperty = IfcComplexProperty;
var IfcComplexPropertyTemplate = /** @class */ (function (_super) {
    __extends(IfcComplexPropertyTemplate, _super);
    function IfcComplexPropertyTemplate(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcComplexPropertyTemplate";
        if (props) {
            _this.HasPropertyTemplates = (_a = props.HasPropertyTemplates) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPropertyTemplate(o); });
        }
        return _this;
    }
    return IfcComplexPropertyTemplate;
}(IfcPropertyTemplate));
exports.IfcComplexPropertyTemplate = IfcComplexPropertyTemplate;
var IfcCompositeCurveSegment = /** @class */ (function (_super) {
    __extends(IfcCompositeCurveSegment, _super);
    function IfcCompositeCurveSegment(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCompositeCurveSegment";
        if (props) {
            _this.ParentCurve = (props.ParentCurve) ? new IfcCurve(props.ParentCurve) : undefined;
        }
        return _this;
    }
    return IfcCompositeCurveSegment;
}(IfcGeometricRepresentationItem));
exports.IfcCompositeCurveSegment = IfcCompositeCurveSegment;
var IfcConic = /** @class */ (function (_super) {
    __extends(IfcConic, _super);
    function IfcConic(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConic";
        return _this;
    }
    return IfcConic;
}(IfcCurve));
exports.IfcConic = IfcConic;
var IfcConnectedFaceSet = /** @class */ (function (_super) {
    __extends(IfcConnectedFaceSet, _super);
    function IfcConnectedFaceSet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConnectedFaceSet";
        if (props) {
            _this.CfsFaces = (_a = props.CfsFaces) === null || _a === void 0 ? void 0 : _a.map(function (o) { return o; });
        }
        return _this;
    }
    return IfcConnectedFaceSet;
}(IfcTopologicalRepresentationItem));
exports.IfcConnectedFaceSet = IfcConnectedFaceSet;
var IfcConnectionCurveGeometry = /** @class */ (function (_super) {
    __extends(IfcConnectionCurveGeometry, _super);
    function IfcConnectionCurveGeometry(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConnectionCurveGeometry";
        return _this;
    }
    return IfcConnectionCurveGeometry;
}(IfcConnectionGeometry));
exports.IfcConnectionCurveGeometry = IfcConnectionCurveGeometry;
var IfcConnectionPointGeometry = /** @class */ (function (_super) {
    __extends(IfcConnectionPointGeometry, _super);
    function IfcConnectionPointGeometry(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConnectionPointGeometry";
        return _this;
    }
    return IfcConnectionPointGeometry;
}(IfcConnectionGeometry));
exports.IfcConnectionPointGeometry = IfcConnectionPointGeometry;
var IfcConnectionSurfaceGeometry = /** @class */ (function (_super) {
    __extends(IfcConnectionSurfaceGeometry, _super);
    function IfcConnectionSurfaceGeometry(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConnectionSurfaceGeometry";
        return _this;
    }
    return IfcConnectionSurfaceGeometry;
}(IfcConnectionGeometry));
exports.IfcConnectionSurfaceGeometry = IfcConnectionSurfaceGeometry;
var IfcConnectionVolumeGeometry = /** @class */ (function (_super) {
    __extends(IfcConnectionVolumeGeometry, _super);
    function IfcConnectionVolumeGeometry(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConnectionVolumeGeometry";
        return _this;
    }
    return IfcConnectionVolumeGeometry;
}(IfcConnectionGeometry));
exports.IfcConnectionVolumeGeometry = IfcConnectionVolumeGeometry;
var IfcConstraint = /** @class */ (function (_super) {
    __extends(IfcConstraint, _super);
    function IfcConstraint(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConstraint";
        return _this;
    }
    return IfcConstraint;
}(Entity));
exports.IfcConstraint = IfcConstraint;
var IfcConstructionResource = /** @class */ (function (_super) {
    __extends(IfcConstructionResource, _super);
    function IfcConstructionResource(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConstructionResource";
        if (props) {
            _this.Usage = (props.Usage) ? new IfcResourceTime(props.Usage) : undefined;
            _this.BaseCosts = (_a = props.BaseCosts) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcAppliedValue(o); });
            _this.BaseQuantity = (props.BaseQuantity) ? new IfcPhysicalQuantity(props.BaseQuantity) : undefined;
        }
        return _this;
    }
    return IfcConstructionResource;
}(IfcResource));
exports.IfcConstructionResource = IfcConstructionResource;
var IfcConstructionResourceType = /** @class */ (function (_super) {
    __extends(IfcConstructionResourceType, _super);
    function IfcConstructionResourceType(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConstructionResourceType";
        if (props) {
            _this.BaseCosts = (_a = props.BaseCosts) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcAppliedValue(o); });
            _this.BaseQuantity = (props.BaseQuantity) ? new IfcPhysicalQuantity(props.BaseQuantity) : undefined;
        }
        return _this;
    }
    return IfcConstructionResourceType;
}(IfcTypeResource));
exports.IfcConstructionResourceType = IfcConstructionResourceType;
var IfcCoordinateOperation = /** @class */ (function (_super) {
    __extends(IfcCoordinateOperation, _super);
    function IfcCoordinateOperation(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCoordinateOperation";
        if (props) {
            _this.TargetCRS = (props.TargetCRS) ? new IfcCoordinateReferenceSystem(props.TargetCRS) : undefined;
        }
        return _this;
    }
    return IfcCoordinateOperation;
}(Entity));
exports.IfcCoordinateOperation = IfcCoordinateOperation;
var IfcCostItem = /** @class */ (function (_super) {
    __extends(IfcCostItem, _super);
    function IfcCostItem(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCostItem";
        if (props) {
            _this.CostValues = (_a = props.CostValues) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCostValue(o); });
            _this.CostQuantities = (_b = props.CostQuantities) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcPhysicalQuantity(o); });
        }
        return _this;
    }
    return IfcCostItem;
}(IfcControl));
exports.IfcCostItem = IfcCostItem;
var IfcCsgPrimitive3D = /** @class */ (function (_super) {
    __extends(IfcCsgPrimitive3D, _super);
    function IfcCsgPrimitive3D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCsgPrimitive3D";
        if (props) {
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
        }
        return _this;
    }
    return IfcCsgPrimitive3D;
}(IfcGeometricRepresentationItem));
exports.IfcCsgPrimitive3D = IfcCsgPrimitive3D;
var IfcCsgSolid = /** @class */ (function (_super) {
    __extends(IfcCsgSolid, _super);
    function IfcCsgSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCsgSolid";
        return _this;
    }
    return IfcCsgSolid;
}(IfcSolidModel));
exports.IfcCsgSolid = IfcCsgSolid;
var IfcCurrencyRelationship = /** @class */ (function (_super) {
    __extends(IfcCurrencyRelationship, _super);
    function IfcCurrencyRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurrencyRelationship";
        if (props) {
            _this.RelatingMonetaryUnit = (props.RelatingMonetaryUnit) ? new IfcMonetaryUnit(props.RelatingMonetaryUnit) : undefined;
            _this.RelatedMonetaryUnit = (props.RelatedMonetaryUnit) ? new IfcMonetaryUnit(props.RelatedMonetaryUnit) : undefined;
            _this.RateSource = (props.RateSource) ? new IfcLibraryInformation(props.RateSource) : undefined;
        }
        return _this;
    }
    return IfcCurrencyRelationship;
}(IfcResourceLevelRelationship));
exports.IfcCurrencyRelationship = IfcCurrencyRelationship;
var IfcCurveBoundedPlane = /** @class */ (function (_super) {
    __extends(IfcCurveBoundedPlane, _super);
    function IfcCurveBoundedPlane(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurveBoundedPlane";
        if (props) {
            _this.BasisSurface = (props.BasisSurface) ? new IfcPlane(props.BasisSurface) : undefined;
            _this.OuterBoundary = (props.OuterBoundary) ? new IfcCurve(props.OuterBoundary) : undefined;
            _this.InnerBoundaries = (_a = props.InnerBoundaries) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCurve(o); });
        }
        return _this;
    }
    return IfcCurveBoundedPlane;
}(IfcBoundedSurface));
exports.IfcCurveBoundedPlane = IfcCurveBoundedPlane;
var IfcCurveBoundedSurface = /** @class */ (function (_super) {
    __extends(IfcCurveBoundedSurface, _super);
    function IfcCurveBoundedSurface(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurveBoundedSurface";
        if (props) {
            _this.BasisSurface = (props.BasisSurface) ? new IfcSurface(props.BasisSurface) : undefined;
            _this.Boundaries = (_a = props.Boundaries) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcBoundaryCurve(o); });
        }
        return _this;
    }
    return IfcCurveBoundedSurface;
}(IfcBoundedSurface));
exports.IfcCurveBoundedSurface = IfcCurveBoundedSurface;
var IfcCurveStyle = /** @class */ (function (_super) {
    __extends(IfcCurveStyle, _super);
    function IfcCurveStyle(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurveStyle";
        return _this;
    }
    return IfcCurveStyle;
}(IfcPresentationStyle));
exports.IfcCurveStyle = IfcCurveStyle;
var IfcCurveStyleFont = /** @class */ (function (_super) {
    __extends(IfcCurveStyleFont, _super);
    function IfcCurveStyleFont(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurveStyleFont";
        if (props) {
            _this.PatternList = (_a = props.PatternList) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCurveStyleFontPattern(o); });
        }
        return _this;
    }
    return IfcCurveStyleFont;
}(IfcPresentationItem));
exports.IfcCurveStyleFont = IfcCurveStyleFont;
var IfcCurveStyleFontAndScaling = /** @class */ (function (_super) {
    __extends(IfcCurveStyleFontAndScaling, _super);
    function IfcCurveStyleFontAndScaling(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCurveStyleFontAndScaling";
        return _this;
    }
    return IfcCurveStyleFontAndScaling;
}(IfcPresentationItem));
exports.IfcCurveStyleFontAndScaling = IfcCurveStyleFontAndScaling;
var IfcDerivedUnit = /** @class */ (function (_super) {
    __extends(IfcDerivedUnit, _super);
    function IfcDerivedUnit(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDerivedUnit";
        if (props) {
            _this.Elements = (_a = props.Elements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcDerivedUnitElement(o); });
        }
        return _this;
    }
    return IfcDerivedUnit;
}(Entity));
exports.IfcDerivedUnit = IfcDerivedUnit;
var IfcDerivedUnitElement = /** @class */ (function (_super) {
    __extends(IfcDerivedUnitElement, _super);
    function IfcDerivedUnitElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDerivedUnitElement";
        if (props) {
            _this.Unit = (props.Unit) ? new IfcNamedUnit(props.Unit) : undefined;
        }
        return _this;
    }
    return IfcDerivedUnitElement;
}(Entity));
exports.IfcDerivedUnitElement = IfcDerivedUnitElement;
var IfcDocumentInformation = /** @class */ (function (_super) {
    __extends(IfcDocumentInformation, _super);
    function IfcDocumentInformation(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDocumentInformation";
        return _this;
    }
    return IfcDocumentInformation;
}(IfcExternalInformation));
exports.IfcDocumentInformation = IfcDocumentInformation;
var IfcDocumentInformationRelationship = /** @class */ (function (_super) {
    __extends(IfcDocumentInformationRelationship, _super);
    function IfcDocumentInformationRelationship(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDocumentInformationRelationship";
        if (props) {
            _this.RelatingDocument = (props.RelatingDocument) ? new IfcDocumentInformation(props.RelatingDocument) : undefined;
            _this.RelatedDocuments = (_a = props.RelatedDocuments) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcDocumentInformation(o); });
        }
        return _this;
    }
    return IfcDocumentInformationRelationship;
}(IfcResourceLevelRelationship));
exports.IfcDocumentInformationRelationship = IfcDocumentInformationRelationship;
var IfcDocumentReference = /** @class */ (function (_super) {
    __extends(IfcDocumentReference, _super);
    function IfcDocumentReference(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDocumentReference";
        if (props) {
            _this.ReferencedDocument = (props.ReferencedDocument) ? new IfcDocumentInformation(props.ReferencedDocument) : undefined;
        }
        return _this;
    }
    return IfcDocumentReference;
}(IfcExternalReference));
exports.IfcDocumentReference = IfcDocumentReference;
var IfcDoorLiningProperties = /** @class */ (function (_super) {
    __extends(IfcDoorLiningProperties, _super);
    function IfcDoorLiningProperties(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDoorLiningProperties";
        if (props) {
            _this.ShapeAspectStyle = props.ShapeAspectStyle;
        }
        return _this;
    }
    return IfcDoorLiningProperties;
}(IfcPreDefinedPropertySet));
exports.IfcDoorLiningProperties = IfcDoorLiningProperties;
var IfcDoorPanelProperties = /** @class */ (function (_super) {
    __extends(IfcDoorPanelProperties, _super);
    function IfcDoorPanelProperties(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcDoorPanelProperties";
        if (props) {
            _this.ShapeAspectStyle = props.ShapeAspectStyle;
        }
        return _this;
    }
    return IfcDoorPanelProperties;
}(IfcPreDefinedPropertySet));
exports.IfcDoorPanelProperties = IfcDoorPanelProperties;
var IfcEdge = /** @class */ (function (_super) {
    __extends(IfcEdge, _super);
    function IfcEdge(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcEdge";
        if (props) {
            _this.EdgeStart = (props.EdgeStart) ? new IfcVertex(props.EdgeStart) : undefined;
            _this.EdgeEnd = (props.EdgeEnd) ? new IfcVertex(props.EdgeEnd) : undefined;
        }
        return _this;
    }
    return IfcEdge;
}(IfcTopologicalRepresentationItem));
exports.IfcEdge = IfcEdge;
var IfcEdgeLoop = /** @class */ (function (_super) {
    __extends(IfcEdgeLoop, _super);
    function IfcEdgeLoop(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcEdgeLoop";
        if (props) {
            _this.EdgeList = (_a = props.EdgeList) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcOrientedEdge(o); });
        }
        return _this;
    }
    return IfcEdgeLoop;
}(IfcLoop));
exports.IfcEdgeLoop = IfcEdgeLoop;
var IfcElementQuantity = /** @class */ (function (_super) {
    __extends(IfcElementQuantity, _super);
    function IfcElementQuantity(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcElementQuantity";
        if (props) {
            _this.Quantities = (_a = props.Quantities) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPhysicalQuantity(o); });
        }
        return _this;
    }
    return IfcElementQuantity;
}(IfcQuantitySet));
exports.IfcElementQuantity = IfcElementQuantity;
var IfcElementarySurface = /** @class */ (function (_super) {
    __extends(IfcElementarySurface, _super);
    function IfcElementarySurface(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcElementarySurface";
        if (props) {
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
        }
        return _this;
    }
    return IfcElementarySurface;
}(IfcSurface));
exports.IfcElementarySurface = IfcElementarySurface;
var IfcEvent = /** @class */ (function (_super) {
    __extends(IfcEvent, _super);
    function IfcEvent(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcEvent";
        if (props) {
            _this.EventOccurenceTime = (props.EventOccurenceTime) ? new IfcEventTime(props.EventOccurenceTime) : undefined;
        }
        return _this;
    }
    return IfcEvent;
}(IfcProcess));
exports.IfcEvent = IfcEvent;
var IfcExtendedProperties = /** @class */ (function (_super) {
    __extends(IfcExtendedProperties, _super);
    function IfcExtendedProperties(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcExtendedProperties";
        if (props) {
            _this.Properties = (_a = props.Properties) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcExtendedProperties;
}(IfcPropertyAbstraction));
exports.IfcExtendedProperties = IfcExtendedProperties;
var IfcExternalReferenceRelationship = /** @class */ (function (_super) {
    __extends(IfcExternalReferenceRelationship, _super);
    function IfcExternalReferenceRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcExternalReferenceRelationship";
        if (props) {
            _this.RelatingReference = (props.RelatingReference) ? new IfcExternalReference(props.RelatingReference) : undefined;
        }
        return _this;
    }
    return IfcExternalReferenceRelationship;
}(IfcResourceLevelRelationship));
exports.IfcExternalReferenceRelationship = IfcExternalReferenceRelationship;
var IfcFaceBasedSurfaceModel = /** @class */ (function (_super) {
    __extends(IfcFaceBasedSurfaceModel, _super);
    function IfcFaceBasedSurfaceModel(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFaceBasedSurfaceModel";
        if (props) {
            _this.FbsmFaces = (_a = props.FbsmFaces) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcConnectedFaceSet(o); });
        }
        return _this;
    }
    return IfcFaceBasedSurfaceModel;
}(IfcGeometricRepresentationItem));
exports.IfcFaceBasedSurfaceModel = IfcFaceBasedSurfaceModel;
var IfcFaceBound = /** @class */ (function (_super) {
    __extends(IfcFaceBound, _super);
    function IfcFaceBound(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFaceBound";
        if (props) {
            _this.Bound = (props.Bound) ? new IfcLoop(props.Bound) : undefined;
        }
        return _this;
    }
    return IfcFaceBound;
}(IfcTopologicalRepresentationItem));
exports.IfcFaceBound = IfcFaceBound;
var IfcFacetedBrepWithVoids = /** @class */ (function (_super) {
    __extends(IfcFacetedBrepWithVoids, _super);
    function IfcFacetedBrepWithVoids(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFacetedBrepWithVoids";
        if (props) {
            _this.Voids = (_a = props.Voids) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcClosedShell(o); });
        }
        return _this;
    }
    return IfcFacetedBrepWithVoids;
}(IfcFacetedBrep));
exports.IfcFacetedBrepWithVoids = IfcFacetedBrepWithVoids;
var IfcFillAreaStyle = /** @class */ (function (_super) {
    __extends(IfcFillAreaStyle, _super);
    function IfcFillAreaStyle(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFillAreaStyle";
        return _this;
    }
    return IfcFillAreaStyle;
}(IfcPresentationStyle));
exports.IfcFillAreaStyle = IfcFillAreaStyle;
var IfcFillAreaStyleHatching = /** @class */ (function (_super) {
    __extends(IfcFillAreaStyleHatching, _super);
    function IfcFillAreaStyleHatching(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFillAreaStyleHatching";
        if (props) {
            _this.HatchLineAppearance = (props.HatchLineAppearance) ? new IfcCurveStyle(props.HatchLineAppearance) : undefined;
            _this.PointOfReferenceHatchLine = (props.PointOfReferenceHatchLine) ? new IfcCartesianPoint(props.PointOfReferenceHatchLine) : undefined;
            _this.PatternStart = (props.PatternStart) ? new IfcCartesianPoint(props.PatternStart) : undefined;
        }
        return _this;
    }
    return IfcFillAreaStyleHatching;
}(IfcGeometricRepresentationItem));
exports.IfcFillAreaStyleHatching = IfcFillAreaStyleHatching;
var IfcFillAreaStyleTiles = /** @class */ (function (_super) {
    __extends(IfcFillAreaStyleTiles, _super);
    function IfcFillAreaStyleTiles(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFillAreaStyleTiles";
        if (props) {
            _this.TilingPattern = (props.TilingPattern) ? new TilingPattern(props.TilingPattern) : undefined;
            _this.Tiles = (_a = props.Tiles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcStyledItem(o); });
        }
        return _this;
    }
    return IfcFillAreaStyleTiles;
}(IfcGeometricRepresentationItem));
exports.IfcFillAreaStyleTiles = IfcFillAreaStyleTiles;
var IfcGeometricRepresentationContext = /** @class */ (function (_super) {
    __extends(IfcGeometricRepresentationContext, _super);
    function IfcGeometricRepresentationContext(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcGeometricRepresentationContext";
        if (props) {
            _this.TrueNorth = (props.TrueNorth) ? new IfcDirection(props.TrueNorth) : undefined;
            _this.HasSubContexts = (_a = props.HasSubContexts) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcGeometricRepresentationSubContext(o); });
            _this.HasCoordinateOperation = (props.HasCoordinateOperation) ? new IfcCoordinateOperation(props.HasCoordinateOperation) : undefined;
        }
        return _this;
    }
    return IfcGeometricRepresentationContext;
}(IfcRepresentationContext));
exports.IfcGeometricRepresentationContext = IfcGeometricRepresentationContext;
var IfcGeometricSet = /** @class */ (function (_super) {
    __extends(IfcGeometricSet, _super);
    function IfcGeometricSet(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcGeometricSet";
        return _this;
    }
    return IfcGeometricSet;
}(IfcGeometricRepresentationItem));
exports.IfcGeometricSet = IfcGeometricSet;
var IfcGridAxis = /** @class */ (function (_super) {
    __extends(IfcGridAxis, _super);
    function IfcGridAxis(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcGridAxis";
        if (props) {
            _this.AxisCurve = (props.AxisCurve) ? new IfcCurve(props.AxisCurve) : undefined;
        }
        return _this;
    }
    return IfcGridAxis;
}(Entity));
exports.IfcGridAxis = IfcGridAxis;
var IfcGridPlacement = /** @class */ (function (_super) {
    __extends(IfcGridPlacement, _super);
    function IfcGridPlacement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcGridPlacement";
        if (props) {
            _this.PlacementLocation = (props.PlacementLocation) ? new IfcVirtualGridIntersection(props.PlacementLocation) : undefined;
        }
        return _this;
    }
    return IfcGridPlacement;
}(IfcObjectPlacement));
exports.IfcGridPlacement = IfcGridPlacement;
var IfcHalfSpaceSolid = /** @class */ (function (_super) {
    __extends(IfcHalfSpaceSolid, _super);
    function IfcHalfSpaceSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcHalfSpaceSolid";
        if (props) {
            _this.BaseSurface = (props.BaseSurface) ? new IfcSurface(props.BaseSurface) : undefined;
        }
        return _this;
    }
    return IfcHalfSpaceSolid;
}(IfcGeometricRepresentationItem));
exports.IfcHalfSpaceSolid = IfcHalfSpaceSolid;
var IfcIndexedColourMap = /** @class */ (function (_super) {
    __extends(IfcIndexedColourMap, _super);
    function IfcIndexedColourMap(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcIndexedColourMap";
        if (props) {
            _this.Colours = (props.Colours) ? new IfcColourRgbList(props.Colours) : undefined;
        }
        return _this;
    }
    return IfcIndexedColourMap;
}(IfcPresentationItem));
exports.IfcIndexedColourMap = IfcIndexedColourMap;
var IfcIndexedPolyCurve = /** @class */ (function (_super) {
    __extends(IfcIndexedPolyCurve, _super);
    function IfcIndexedPolyCurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcIndexedPolyCurve";
        if (props) {
            _this.Points = (props.Points) ? new IfcCartesianPointList(props.Points) : undefined;
        }
        return _this;
    }
    return IfcIndexedPolyCurve;
}(IfcBoundedCurve));
exports.IfcIndexedPolyCurve = IfcIndexedPolyCurve;
var IfcInventory = /** @class */ (function (_super) {
    __extends(IfcInventory, _super);
    function IfcInventory(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcInventory";
        if (props) {
            _this.ResponsiblePersons = (_a = props.ResponsiblePersons) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPerson(o); });
            _this.CurrentValue = (props.CurrentValue) ? new IfcCostValue(props.CurrentValue) : undefined;
            _this.OriginalValue = (props.OriginalValue) ? new IfcCostValue(props.OriginalValue) : undefined;
        }
        return _this;
    }
    return IfcInventory;
}(IfcGroup));
exports.IfcInventory = IfcInventory;
var IfcIrregularTimeSeriesValue = /** @class */ (function (_super) {
    __extends(IfcIrregularTimeSeriesValue, _super);
    function IfcIrregularTimeSeriesValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcIrregularTimeSeriesValue";
        return _this;
    }
    return IfcIrregularTimeSeriesValue;
}(Entity));
exports.IfcIrregularTimeSeriesValue = IfcIrregularTimeSeriesValue;
var IfcLagTime = /** @class */ (function (_super) {
    __extends(IfcLagTime, _super);
    function IfcLagTime(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLagTime";
        return _this;
    }
    return IfcLagTime;
}(IfcSchedulingTime));
exports.IfcLagTime = IfcLagTime;
var IfcLibraryInformation = /** @class */ (function (_super) {
    __extends(IfcLibraryInformation, _super);
    function IfcLibraryInformation(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLibraryInformation";
        return _this;
    }
    return IfcLibraryInformation;
}(IfcExternalInformation));
exports.IfcLibraryInformation = IfcLibraryInformation;
var IfcLibraryReference = /** @class */ (function (_super) {
    __extends(IfcLibraryReference, _super);
    function IfcLibraryReference(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLibraryReference";
        if (props) {
            _this.ReferencedLibrary = (props.ReferencedLibrary) ? new IfcLibraryInformation(props.ReferencedLibrary) : undefined;
        }
        return _this;
    }
    return IfcLibraryReference;
}(IfcExternalReference));
exports.IfcLibraryReference = IfcLibraryReference;
var IfcLightIntensityDistribution = /** @class */ (function (_super) {
    __extends(IfcLightIntensityDistribution, _super);
    function IfcLightIntensityDistribution(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightIntensityDistribution";
        if (props) {
            _this.DistributionData = (_a = props.DistributionData) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcLightDistributionData(o); });
        }
        return _this;
    }
    return IfcLightIntensityDistribution;
}(Entity));
exports.IfcLightIntensityDistribution = IfcLightIntensityDistribution;
var IfcLightSource = /** @class */ (function (_super) {
    __extends(IfcLightSource, _super);
    function IfcLightSource(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightSource";
        if (props) {
            _this.LightColour = (props.LightColour) ? new IfcColourRgb(props.LightColour) : undefined;
        }
        return _this;
    }
    return IfcLightSource;
}(IfcGeometricRepresentationItem));
exports.IfcLightSource = IfcLightSource;
var IfcLine = /** @class */ (function (_super) {
    __extends(IfcLine, _super);
    function IfcLine(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLine";
        if (props) {
            _this.Pnt = (props.Pnt) ? new IfcCartesianPoint(props.Pnt) : undefined;
            _this.Dir = (props.Dir) ? new IfcVector(props.Dir) : undefined;
        }
        return _this;
    }
    return IfcLine;
}(IfcCurve));
exports.IfcLine = IfcLine;
var IfcLocalPlacement = /** @class */ (function (_super) {
    __extends(IfcLocalPlacement, _super);
    function IfcLocalPlacement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLocalPlacement";
        if (props) {
            _this.PlacementRelTo = (props.PlacementRelTo) ? new IfcObjectPlacement(props.PlacementRelTo) : undefined;
        }
        return _this;
    }
    return IfcLocalPlacement;
}(IfcObjectPlacement));
exports.IfcLocalPlacement = IfcLocalPlacement;
var IfcManifoldSolidBrep = /** @class */ (function (_super) {
    __extends(IfcManifoldSolidBrep, _super);
    function IfcManifoldSolidBrep(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcManifoldSolidBrep";
        if (props) {
            _this.Outer = (props.Outer) ? new IfcClosedShell(props.Outer) : undefined;
        }
        return _this;
    }
    return IfcManifoldSolidBrep;
}(IfcSolidModel));
exports.IfcManifoldSolidBrep = IfcManifoldSolidBrep;
var IfcMaterialClassificationRelationship = /** @class */ (function (_super) {
    __extends(IfcMaterialClassificationRelationship, _super);
    function IfcMaterialClassificationRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialClassificationRelationship";
        if (props) {
            _this.ClassifiedMaterial = (props.ClassifiedMaterial) ? new IfcMaterial(props.ClassifiedMaterial) : undefined;
        }
        return _this;
    }
    return IfcMaterialClassificationRelationship;
}(Entity));
exports.IfcMaterialClassificationRelationship = IfcMaterialClassificationRelationship;
var IfcMaterialDefinition = /** @class */ (function (_super) {
    __extends(IfcMaterialDefinition, _super);
    function IfcMaterialDefinition(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialDefinition";
        if (props) {
            _this.HasProperties = (_a = props.HasProperties) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcMaterialDefinition;
}(Entity));
exports.IfcMaterialDefinition = IfcMaterialDefinition;
var IfcMaterialLayerSetUsage = /** @class */ (function (_super) {
    __extends(IfcMaterialLayerSetUsage, _super);
    function IfcMaterialLayerSetUsage(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialLayerSetUsage";
        if (props) {
            _this.ForLayerSet = (props.ForLayerSet) ? new IfcMaterialLayerSet(props.ForLayerSet) : undefined;
        }
        return _this;
    }
    return IfcMaterialLayerSetUsage;
}(IfcMaterialUsageDefinition));
exports.IfcMaterialLayerSetUsage = IfcMaterialLayerSetUsage;
var IfcMaterialList = /** @class */ (function (_super) {
    __extends(IfcMaterialList, _super);
    function IfcMaterialList(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialList";
        if (props) {
            _this.Materials = (_a = props.Materials) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcMaterial(o); });
        }
        return _this;
    }
    return IfcMaterialList;
}(Entity));
exports.IfcMaterialList = IfcMaterialList;
var IfcMaterialProfileSetUsage = /** @class */ (function (_super) {
    __extends(IfcMaterialProfileSetUsage, _super);
    function IfcMaterialProfileSetUsage(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialProfileSetUsage";
        if (props) {
            _this.ForProfileSet = (props.ForProfileSet) ? new IfcMaterialProfileSet(props.ForProfileSet) : undefined;
        }
        return _this;
    }
    return IfcMaterialProfileSetUsage;
}(IfcMaterialUsageDefinition));
exports.IfcMaterialProfileSetUsage = IfcMaterialProfileSetUsage;
var IfcMaterialRelationship = /** @class */ (function (_super) {
    __extends(IfcMaterialRelationship, _super);
    function IfcMaterialRelationship(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialRelationship";
        if (props) {
            _this.RelatingMaterial = (props.RelatingMaterial) ? new IfcMaterial(props.RelatingMaterial) : undefined;
            _this.RelatedMaterials = (_a = props.RelatedMaterials) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcMaterial(o); });
        }
        return _this;
    }
    return IfcMaterialRelationship;
}(IfcResourceLevelRelationship));
exports.IfcMaterialRelationship = IfcMaterialRelationship;
var IfcMeasureWithUnit = /** @class */ (function (_super) {
    __extends(IfcMeasureWithUnit, _super);
    function IfcMeasureWithUnit(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMeasureWithUnit";
        return _this;
    }
    return IfcMeasureWithUnit;
}(Entity));
exports.IfcMeasureWithUnit = IfcMeasureWithUnit;
var IfcNamedUnit = /** @class */ (function (_super) {
    __extends(IfcNamedUnit, _super);
    function IfcNamedUnit(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcNamedUnit";
        if (props) {
            _this.Dimensions = (props.Dimensions) ? new IfcDimensionalExponents(props.Dimensions) : undefined;
        }
        return _this;
    }
    return IfcNamedUnit;
}(Entity));
exports.IfcNamedUnit = IfcNamedUnit;
var IfcOffsetCurve2D = /** @class */ (function (_super) {
    __extends(IfcOffsetCurve2D, _super);
    function IfcOffsetCurve2D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOffsetCurve2D";
        if (props) {
            _this.BasisCurve = (props.BasisCurve) ? new IfcCurve(props.BasisCurve) : undefined;
        }
        return _this;
    }
    return IfcOffsetCurve2D;
}(IfcCurve));
exports.IfcOffsetCurve2D = IfcOffsetCurve2D;
var IfcOffsetCurve3D = /** @class */ (function (_super) {
    __extends(IfcOffsetCurve3D, _super);
    function IfcOffsetCurve3D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOffsetCurve3D";
        if (props) {
            _this.BasisCurve = (props.BasisCurve) ? new IfcCurve(props.BasisCurve) : undefined;
            _this.RefDirection = (props.RefDirection) ? new IfcDirection(props.RefDirection) : undefined;
        }
        return _this;
    }
    return IfcOffsetCurve3D;
}(IfcCurve));
exports.IfcOffsetCurve3D = IfcOffsetCurve3D;
var IfcOpeningElement = /** @class */ (function (_super) {
    __extends(IfcOpeningElement, _super);
    function IfcOpeningElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOpeningElement";
        if (props) {
            _this.HasFillings = (props.HasFillings) ? new IfcRelFillsElement(props.HasFillings) : undefined;
        }
        return _this;
    }
    return IfcOpeningElement;
}(IfcFeatureElementSubtraction));
exports.IfcOpeningElement = IfcOpeningElement;
var IfcOrganization = /** @class */ (function (_super) {
    __extends(IfcOrganization, _super);
    function IfcOrganization(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOrganization";
        if (props) {
            _this.Roles = (_a = props.Roles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcActorRole(o); });
            _this.Addresses = (_b = props.Addresses) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcAddress(o); });
        }
        return _this;
    }
    return IfcOrganization;
}(Entity));
exports.IfcOrganization = IfcOrganization;
var IfcOrganizationRelationship = /** @class */ (function (_super) {
    __extends(IfcOrganizationRelationship, _super);
    function IfcOrganizationRelationship(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOrganizationRelationship";
        if (props) {
            _this.RelatingOrganization = (props.RelatingOrganization) ? new IfcOrganization(props.RelatingOrganization) : undefined;
            _this.RelatedOrganizations = (_a = props.RelatedOrganizations) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcOrganization(o); });
        }
        return _this;
    }
    return IfcOrganizationRelationship;
}(IfcResourceLevelRelationship));
exports.IfcOrganizationRelationship = IfcOrganizationRelationship;
var IfcOrientedEdge = /** @class */ (function (_super) {
    __extends(IfcOrientedEdge, _super);
    function IfcOrientedEdge() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcOrientedEdge;
}(IfcOrientedEdge));
exports.IfcOrientedEdge = IfcOrientedEdge;
-temp;
{
    EdgeElement: IfcEdge;
    constructor(props ?  : IfcOrientedEdge);
    {
        _this = _super.call(this, props) || this;
        this["@class"] = ".IfcOrientedEdge";
        if (props) {
            this.EdgeElement = (props.EdgeElement) ? new IfcEdge(props.EdgeElement) : undefined;
        }
    }
}
var IfcOwnerHistory = /** @class */ (function (_super) {
    __extends(IfcOwnerHistory, _super);
    function IfcOwnerHistory(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcOwnerHistory";
        if (props) {
            _this.OwningUser = (props.OwningUser) ? new IfcPersonAndOrganization(props.OwningUser) : undefined;
            _this.OwningApplication = (props.OwningApplication) ? new IfcApplication(props.OwningApplication) : undefined;
            _this.LastModifyingUser = (props.LastModifyingUser) ? new IfcPersonAndOrganization(props.LastModifyingUser) : undefined;
            _this.LastModifyingApplication = (props.LastModifyingApplication) ? new IfcApplication(props.LastModifyingApplication) : undefined;
        }
        return _this;
    }
    return IfcOwnerHistory;
}(Entity));
exports.IfcOwnerHistory = IfcOwnerHistory;
var IfcPath = /** @class */ (function (_super) {
    __extends(IfcPath, _super);
    function IfcPath(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPath";
        if (props) {
            _this.EdgeList = (_a = props.EdgeList) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcOrientedEdge(o); });
        }
        return _this;
    }
    return IfcPath;
}(IfcTopologicalRepresentationItem));
exports.IfcPath = IfcPath;
var IfcPcurve = /** @class */ (function (_super) {
    __extends(IfcPcurve, _super);
    function IfcPcurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPcurve";
        if (props) {
            _this.BasisSurface = (props.BasisSurface) ? new IfcSurface(props.BasisSurface) : undefined;
            _this.ReferenceCurve = (props.ReferenceCurve) ? new IfcCurve(props.ReferenceCurve) : undefined;
        }
        return _this;
    }
    return IfcPcurve;
}(IfcCurve));
exports.IfcPcurve = IfcPcurve;
var IfcPermeableCoveringProperties = /** @class */ (function (_super) {
    __extends(IfcPermeableCoveringProperties, _super);
    function IfcPermeableCoveringProperties(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPermeableCoveringProperties";
        if (props) {
            _this.ShapeAspectStyle = props.ShapeAspectStyle;
        }
        return _this;
    }
    return IfcPermeableCoveringProperties;
}(IfcPreDefinedPropertySet));
exports.IfcPermeableCoveringProperties = IfcPermeableCoveringProperties;
var IfcPerson = /** @class */ (function (_super) {
    __extends(IfcPerson, _super);
    function IfcPerson(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPerson";
        if (props) {
            _this.Roles = (_a = props.Roles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcActorRole(o); });
            _this.Addresses = (_b = props.Addresses) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcAddress(o); });
        }
        return _this;
    }
    return IfcPerson;
}(Entity));
exports.IfcPerson = IfcPerson;
var IfcPersonAndOrganization = /** @class */ (function (_super) {
    __extends(IfcPersonAndOrganization, _super);
    function IfcPersonAndOrganization(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPersonAndOrganization";
        if (props) {
            _this.ThePerson = (props.ThePerson) ? new IfcPerson(props.ThePerson) : undefined;
            _this.TheOrganization = (props.TheOrganization) ? new IfcOrganization(props.TheOrganization) : undefined;
            _this.Roles = (_a = props.Roles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcActorRole(o); });
        }
        return _this;
    }
    return IfcPersonAndOrganization;
}(Entity));
exports.IfcPersonAndOrganization = IfcPersonAndOrganization;
var IfcPhysicalComplexQuantity = /** @class */ (function (_super) {
    __extends(IfcPhysicalComplexQuantity, _super);
    function IfcPhysicalComplexQuantity(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPhysicalComplexQuantity";
        if (props) {
            _this.HasQuantities = (_a = props.HasQuantities) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPhysicalQuantity(o); });
        }
        return _this;
    }
    return IfcPhysicalComplexQuantity;
}(IfcPhysicalQuantity));
exports.IfcPhysicalComplexQuantity = IfcPhysicalComplexQuantity;
var IfcPhysicalSimpleQuantity = /** @class */ (function (_super) {
    __extends(IfcPhysicalSimpleQuantity, _super);
    function IfcPhysicalSimpleQuantity(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPhysicalSimpleQuantity";
        if (props) {
            _this.Unit = (props.Unit) ? new IfcNamedUnit(props.Unit) : undefined;
        }
        return _this;
    }
    return IfcPhysicalSimpleQuantity;
}(IfcPhysicalQuantity));
exports.IfcPhysicalSimpleQuantity = IfcPhysicalSimpleQuantity;
var IfcPlacement = /** @class */ (function (_super) {
    __extends(IfcPlacement, _super);
    function IfcPlacement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPlacement";
        if (props) {
            _this.Location = (props.Location) ? new IfcCartesianPoint(props.Location) : undefined;
        }
        return _this;
    }
    return IfcPlacement;
}(IfcGeometricRepresentationItem));
exports.IfcPlacement = IfcPlacement;
var IfcPlanarBox = /** @class */ (function (_super) {
    __extends(IfcPlanarBox, _super);
    function IfcPlanarBox(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPlanarBox";
        return _this;
    }
    return IfcPlanarBox;
}(IfcPlanarExtent));
exports.IfcPlanarBox = IfcPlanarBox;
var IfcPointOnCurve = /** @class */ (function (_super) {
    __extends(IfcPointOnCurve, _super);
    function IfcPointOnCurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPointOnCurve";
        if (props) {
            _this.BasisCurve = (props.BasisCurve) ? new IfcCurve(props.BasisCurve) : undefined;
        }
        return _this;
    }
    return IfcPointOnCurve;
}(IfcPoint));
exports.IfcPointOnCurve = IfcPointOnCurve;
var IfcPointOnSurface = /** @class */ (function (_super) {
    __extends(IfcPointOnSurface, _super);
    function IfcPointOnSurface(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPointOnSurface";
        if (props) {
            _this.BasisSurface = (props.BasisSurface) ? new IfcSurface(props.BasisSurface) : undefined;
        }
        return _this;
    }
    return IfcPointOnSurface;
}(IfcPoint));
exports.IfcPointOnSurface = IfcPointOnSurface;
var IfcPolyLoop = /** @class */ (function (_super) {
    __extends(IfcPolyLoop, _super);
    function IfcPolyLoop(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPolyLoop";
        if (props) {
            _this.Polygon = (_a = props.Polygon) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCartesianPoint(o); });
        }
        return _this;
    }
    return IfcPolyLoop;
}(IfcLoop));
exports.IfcPolyLoop = IfcPolyLoop;
var IfcPolyline = /** @class */ (function (_super) {
    __extends(IfcPolyline, _super);
    function IfcPolyline(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPolyline";
        if (props) {
            _this.Points = (_a = props.Points) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCartesianPoint(o); });
        }
        return _this;
    }
    return IfcPolyline;
}(IfcBoundedCurve));
exports.IfcPolyline = IfcPolyline;
var IfcPresentationLayerAssignment = /** @class */ (function (_super) {
    __extends(IfcPresentationLayerAssignment, _super);
    function IfcPresentationLayerAssignment(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPresentationLayerAssignment";
        return _this;
    }
    return IfcPresentationLayerAssignment;
}(Entity));
exports.IfcPresentationLayerAssignment = IfcPresentationLayerAssignment;
var IfcPresentationStyleAssignment = /** @class */ (function (_super) {
    __extends(IfcPresentationStyleAssignment, _super);
    function IfcPresentationStyleAssignment(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPresentationStyleAssignment";
        return _this;
    }
    return IfcPresentationStyleAssignment;
}(Entity));
exports.IfcPresentationStyleAssignment = IfcPresentationStyleAssignment;
var IfcProjectedCRS = /** @class */ (function (_super) {
    __extends(IfcProjectedCRS, _super);
    function IfcProjectedCRS(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcProjectedCRS";
        if (props) {
            _this.MapUnit = (props.MapUnit) ? new IfcNamedUnit(props.MapUnit) : undefined;
        }
        return _this;
    }
    return IfcProjectedCRS;
}(IfcCoordinateReferenceSystem));
exports.IfcProjectedCRS = IfcProjectedCRS;
var IfcPropertyBoundedValue = /** @class */ (function (_super) {
    __extends(IfcPropertyBoundedValue, _super);
    function IfcPropertyBoundedValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyBoundedValue";
        return _this;
    }
    return IfcPropertyBoundedValue;
}(IfcSimpleProperty));
exports.IfcPropertyBoundedValue = IfcPropertyBoundedValue;
var IfcPropertyDependencyRelationship = /** @class */ (function (_super) {
    __extends(IfcPropertyDependencyRelationship, _super);
    function IfcPropertyDependencyRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyDependencyRelationship";
        if (props) {
            _this.DependingProperty = (props.DependingProperty) ? new IfcProperty(props.DependingProperty) : undefined;
            _this.DependantProperty = (props.DependantProperty) ? new IfcProperty(props.DependantProperty) : undefined;
        }
        return _this;
    }
    return IfcPropertyDependencyRelationship;
}(IfcResourceLevelRelationship));
exports.IfcPropertyDependencyRelationship = IfcPropertyDependencyRelationship;
var IfcPropertyEnumeratedValue = /** @class */ (function (_super) {
    __extends(IfcPropertyEnumeratedValue, _super);
    function IfcPropertyEnumeratedValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyEnumeratedValue";
        if (props) {
            _this.EnumerationReference = (props.EnumerationReference) ? new IfcPropertyEnumeration(props.EnumerationReference) : undefined;
        }
        return _this;
    }
    return IfcPropertyEnumeratedValue;
}(IfcSimpleProperty));
exports.IfcPropertyEnumeratedValue = IfcPropertyEnumeratedValue;
var IfcPropertyEnumeration = /** @class */ (function (_super) {
    __extends(IfcPropertyEnumeration, _super);
    function IfcPropertyEnumeration(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyEnumeration";
        return _this;
    }
    return IfcPropertyEnumeration;
}(IfcPropertyAbstraction));
exports.IfcPropertyEnumeration = IfcPropertyEnumeration;
var IfcPropertyListValue = /** @class */ (function (_super) {
    __extends(IfcPropertyListValue, _super);
    function IfcPropertyListValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyListValue";
        return _this;
    }
    return IfcPropertyListValue;
}(IfcSimpleProperty));
exports.IfcPropertyListValue = IfcPropertyListValue;
var IfcPropertyReferenceValue = /** @class */ (function (_super) {
    __extends(IfcPropertyReferenceValue, _super);
    function IfcPropertyReferenceValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyReferenceValue";
        return _this;
    }
    return IfcPropertyReferenceValue;
}(IfcSimpleProperty));
exports.IfcPropertyReferenceValue = IfcPropertyReferenceValue;
var IfcPropertySet = /** @class */ (function (_super) {
    __extends(IfcPropertySet, _super);
    function IfcPropertySet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertySet";
        if (props) {
            _this.HasProperties = (_a = props.HasProperties) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcPropertySet;
}(IfcPropertySetDefinition));
exports.IfcPropertySet = IfcPropertySet;
var IfcPropertySingleValue = /** @class */ (function (_super) {
    __extends(IfcPropertySingleValue, _super);
    function IfcPropertySingleValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertySingleValue";
        return _this;
    }
    return IfcPropertySingleValue;
}(IfcSimpleProperty));
exports.IfcPropertySingleValue = IfcPropertySingleValue;
var IfcPropertyTableValue = /** @class */ (function (_super) {
    __extends(IfcPropertyTableValue, _super);
    function IfcPropertyTableValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPropertyTableValue";
        return _this;
    }
    return IfcPropertyTableValue;
}(IfcSimpleProperty));
exports.IfcPropertyTableValue = IfcPropertyTableValue;
var IfcRationalBSplineSurfaceWithKnots = /** @class */ (function (_super) {
    __extends(IfcRationalBSplineSurfaceWithKnots, _super);
    function IfcRationalBSplineSurfaceWithKnots() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcRationalBSplineSurfaceWithKnots;
}(IfcBSplineSurfaceWithKnots));
exports.IfcRationalBSplineSurfaceWithKnots = IfcRationalBSplineSurfaceWithKnots;
-wrapper[];
constructor(props ?  : IfcRationalBSplineSurfaceWithKnots);
{
    _this = _super.call(this, props) || this;
    this["@class"] = ".IfcRationalBSplineSurfaceWithKnots";
    if (props) {
        this.WeightsData = (_d = props.WeightsData) === null || _d === void 0 ? void 0 : _d.map(function (o) { return new IfcReal - wrapper(o); });
    }
}
var IfcRectangularTrimmedSurface = /** @class */ (function (_super) {
    __extends(IfcRectangularTrimmedSurface, _super);
    function IfcRectangularTrimmedSurface(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRectangularTrimmedSurface";
        if (props) {
            _this.BasisSurface = (props.BasisSurface) ? new IfcSurface(props.BasisSurface) : undefined;
        }
        return _this;
    }
    return IfcRectangularTrimmedSurface;
}(IfcBoundedSurface));
exports.IfcRectangularTrimmedSurface = IfcRectangularTrimmedSurface;
var IfcReference = /** @class */ (function (_super) {
    __extends(IfcReference, _super);
    function IfcReference(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcReference";
        if (props) {
            _this.InnerReference = (props.InnerReference) ? new IfcReference(props.InnerReference) : undefined;
        }
        return _this;
    }
    return IfcReference;
}(Entity));
exports.IfcReference = IfcReference;
var IfcReinforcementDefinitionProperties = /** @class */ (function (_super) {
    __extends(IfcReinforcementDefinitionProperties, _super);
    function IfcReinforcementDefinitionProperties(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcReinforcementDefinitionProperties";
        if (props) {
            _this.ReinforcementSectionDefinitions = (_a = props.ReinforcementSectionDefinitions) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcSectionReinforcementProperties(o); });
        }
        return _this;
    }
    return IfcReinforcementDefinitionProperties;
}(IfcPreDefinedPropertySet));
exports.IfcReinforcementDefinitionProperties = IfcReinforcementDefinitionProperties;
var IfcReinforcingBarType = /** @class */ (function (_super) {
    __extends(IfcReinforcingBarType, _super);
    function IfcReinforcingBarType(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcReinforcingBarType";
        return _this;
    }
    return IfcReinforcingBarType;
}(IfcReinforcingElementType));
exports.IfcReinforcingBarType = IfcReinforcingBarType;
var IfcReinforcingMeshType = /** @class */ (function (_super) {
    __extends(IfcReinforcingMeshType, _super);
    function IfcReinforcingMeshType(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcReinforcingMeshType";
        return _this;
    }
    return IfcReinforcingMeshType;
}(IfcReinforcingElementType));
exports.IfcReinforcingMeshType = IfcReinforcingMeshType;
var IfcRelAggregates = /** @class */ (function (_super) {
    __extends(IfcRelAggregates, _super);
    function IfcRelAggregates(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAggregates";
        if (props) {
            _this.RelatedObjects = (_a = props.RelatedObjects) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcObjectDefinition(o); });
        }
        return _this;
    }
    return IfcRelAggregates;
}(IfcRelDecomposes));
exports.IfcRelAggregates = IfcRelAggregates;
var IfcRelAssigns = /** @class */ (function (_super) {
    __extends(IfcRelAssigns, _super);
    function IfcRelAssigns(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssigns";
        if (props) {
            _this.RelatedObjects = (_a = props.RelatedObjects) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcObjectDefinition(o); });
        }
        return _this;
    }
    return IfcRelAssigns;
}(IfcRelationship));
exports.IfcRelAssigns = IfcRelAssigns;
var IfcRelAssociates = /** @class */ (function (_super) {
    __extends(IfcRelAssociates, _super);
    function IfcRelAssociates(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociates";
        return _this;
    }
    return IfcRelAssociates;
}(IfcRelationship));
exports.IfcRelAssociates = IfcRelAssociates;
var IfcRelConnectsElements = /** @class */ (function (_super) {
    __extends(IfcRelConnectsElements, _super);
    function IfcRelConnectsElements(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsElements";
        if (props) {
            _this.ConnectionGeometry = (props.ConnectionGeometry) ? new IfcConnectionGeometry(props.ConnectionGeometry) : undefined;
            _this.RelatingElement = (props.RelatingElement) ? new IfcElement(props.RelatingElement) : undefined;
            _this.RelatedElement = (props.RelatedElement) ? new IfcElement(props.RelatedElement) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsElements;
}(IfcRelConnects));
exports.IfcRelConnectsElements = IfcRelConnectsElements;
var IfcRelConnectsPortToElement = /** @class */ (function (_super) {
    __extends(IfcRelConnectsPortToElement, _super);
    function IfcRelConnectsPortToElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsPortToElement";
        if (props) {
            _this.RelatingPort = (props.RelatingPort) ? new IfcPort(props.RelatingPort) : undefined;
            _this.RelatedElement = (props.RelatedElement) ? new IfcDistributionElement(props.RelatedElement) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsPortToElement;
}(IfcRelConnects));
exports.IfcRelConnectsPortToElement = IfcRelConnectsPortToElement;
var IfcRelConnectsPorts = /** @class */ (function (_super) {
    __extends(IfcRelConnectsPorts, _super);
    function IfcRelConnectsPorts(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsPorts";
        if (props) {
            _this.RelatingPort = (props.RelatingPort) ? new IfcPort(props.RelatingPort) : undefined;
            _this.RelatedPort = (props.RelatedPort) ? new IfcPort(props.RelatedPort) : undefined;
            _this.RealizingElement = (props.RealizingElement) ? new IfcElement(props.RealizingElement) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsPorts;
}(IfcRelConnects));
exports.IfcRelConnectsPorts = IfcRelConnectsPorts;
var IfcRelConnectsStructuralActivity = /** @class */ (function (_super) {
    __extends(IfcRelConnectsStructuralActivity, _super);
    function IfcRelConnectsStructuralActivity(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsStructuralActivity";
        if (props) {
            _this.RelatedStructuralActivity = (props.RelatedStructuralActivity) ? new IfcStructuralActivity(props.RelatedStructuralActivity) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsStructuralActivity;
}(IfcRelConnects));
exports.IfcRelConnectsStructuralActivity = IfcRelConnectsStructuralActivity;
var IfcRelConnectsStructuralMember = /** @class */ (function (_super) {
    __extends(IfcRelConnectsStructuralMember, _super);
    function IfcRelConnectsStructuralMember(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsStructuralMember";
        if (props) {
            _this.RelatingStructuralMember = (props.RelatingStructuralMember) ? new IfcStructuralMember(props.RelatingStructuralMember) : undefined;
            _this.RelatedStructuralConnection = (props.RelatedStructuralConnection) ? new IfcStructuralConnection(props.RelatedStructuralConnection) : undefined;
            _this.AppliedCondition = (props.AppliedCondition) ? new IfcBoundaryCondition(props.AppliedCondition) : undefined;
            _this.AdditionalConditions = (props.AdditionalConditions) ? new IfcStructuralConnectionCondition(props.AdditionalConditions) : undefined;
            _this.ConditionCoordinateSystem = (props.ConditionCoordinateSystem) ? new IfcAxis2Placement3D(props.ConditionCoordinateSystem) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsStructuralMember;
}(IfcRelConnects));
exports.IfcRelConnectsStructuralMember = IfcRelConnectsStructuralMember;
var IfcRelContainedInSpatialStructure = /** @class */ (function (_super) {
    __extends(IfcRelContainedInSpatialStructure, _super);
    function IfcRelContainedInSpatialStructure(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelContainedInSpatialStructure";
        if (props) {
            _this.RelatedElements = (_a = props.RelatedElements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProduct(o); });
        }
        return _this;
    }
    return IfcRelContainedInSpatialStructure;
}(IfcRelConnects));
exports.IfcRelContainedInSpatialStructure = IfcRelContainedInSpatialStructure;
var IfcRelCoversBldgElements = /** @class */ (function (_super) {
    __extends(IfcRelCoversBldgElements, _super);
    function IfcRelCoversBldgElements(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelCoversBldgElements";
        if (props) {
            _this.RelatingBuildingElement = (props.RelatingBuildingElement) ? new IfcElement(props.RelatingBuildingElement) : undefined;
            _this.RelatedCoverings = (_a = props.RelatedCoverings) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCovering(o); });
        }
        return _this;
    }
    return IfcRelCoversBldgElements;
}(IfcRelConnects));
exports.IfcRelCoversBldgElements = IfcRelCoversBldgElements;
var IfcRelCoversSpaces = /** @class */ (function (_super) {
    __extends(IfcRelCoversSpaces, _super);
    function IfcRelCoversSpaces(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelCoversSpaces";
        if (props) {
            _this.RelatingSpace = (props.RelatingSpace) ? new IfcSpace(props.RelatingSpace) : undefined;
            _this.RelatedCoverings = (_a = props.RelatedCoverings) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCovering(o); });
        }
        return _this;
    }
    return IfcRelCoversSpaces;
}(IfcRelConnects));
exports.IfcRelCoversSpaces = IfcRelCoversSpaces;
var IfcRelDeclares = /** @class */ (function (_super) {
    __extends(IfcRelDeclares, _super);
    function IfcRelDeclares(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelDeclares";
        return _this;
    }
    return IfcRelDeclares;
}(IfcRelationship));
exports.IfcRelDeclares = IfcRelDeclares;
var IfcRelDefinesByProperties = /** @class */ (function (_super) {
    __extends(IfcRelDefinesByProperties, _super);
    function IfcRelDefinesByProperties(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelDefinesByProperties";
        return _this;
    }
    return IfcRelDefinesByProperties;
}(IfcRelDefines));
exports.IfcRelDefinesByProperties = IfcRelDefinesByProperties;
var IfcRelDefinesByTemplate = /** @class */ (function (_super) {
    __extends(IfcRelDefinesByTemplate, _super);
    function IfcRelDefinesByTemplate(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelDefinesByTemplate";
        if (props) {
            _this.RelatedPropertySets = (_a = props.RelatedPropertySets) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPropertySetDefinition(o); });
            _this.RelatingTemplate = (_b = props.RelatingTemplate) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcPropertyTemplate(o); });
        }
        return _this;
    }
    return IfcRelDefinesByTemplate;
}(IfcRelDefines));
exports.IfcRelDefinesByTemplate = IfcRelDefinesByTemplate;
var IfcRelDefinesByType = /** @class */ (function (_super) {
    __extends(IfcRelDefinesByType, _super);
    function IfcRelDefinesByType(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelDefinesByType";
        if (props) {
            _this.RelatingType = (props.RelatingType) ? new IfcTypeObject(props.RelatingType) : undefined;
        }
        return _this;
    }
    return IfcRelDefinesByType;
}(IfcRelDefines));
exports.IfcRelDefinesByType = IfcRelDefinesByType;
var IfcRelFillsElement = /** @class */ (function (_super) {
    __extends(IfcRelFillsElement, _super);
    function IfcRelFillsElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelFillsElement";
        if (props) {
            _this.RelatedBuildingElement = (props.RelatedBuildingElement) ? new IfcElement(props.RelatedBuildingElement) : undefined;
        }
        return _this;
    }
    return IfcRelFillsElement;
}(IfcRelConnects));
exports.IfcRelFillsElement = IfcRelFillsElement;
var IfcRelFlowControlElements = /** @class */ (function (_super) {
    __extends(IfcRelFlowControlElements, _super);
    function IfcRelFlowControlElements(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelFlowControlElements";
        if (props) {
            _this.RelatedControlElements = (_a = props.RelatedControlElements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcDistributionControlElement(o); });
            _this.RelatingFlowElement = (props.RelatingFlowElement) ? new IfcDistributionFlowElement(props.RelatingFlowElement) : undefined;
        }
        return _this;
    }
    return IfcRelFlowControlElements;
}(IfcRelConnects));
exports.IfcRelFlowControlElements = IfcRelFlowControlElements;
var IfcRelInterferesElements = /** @class */ (function (_super) {
    __extends(IfcRelInterferesElements, _super);
    function IfcRelInterferesElements(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelInterferesElements";
        if (props) {
            _this.RelatingElement = (props.RelatingElement) ? new IfcElement(props.RelatingElement) : undefined;
            _this.RelatedElement = (props.RelatedElement) ? new IfcElement(props.RelatedElement) : undefined;
            _this.InterferenceGeometry = (props.InterferenceGeometry) ? new IfcConnectionGeometry(props.InterferenceGeometry) : undefined;
        }
        return _this;
    }
    return IfcRelInterferesElements;
}(IfcRelConnects));
exports.IfcRelInterferesElements = IfcRelInterferesElements;
var IfcRelNests = /** @class */ (function (_super) {
    __extends(IfcRelNests, _super);
    function IfcRelNests(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelNests";
        if (props) {
            _this.RelatedObjects = (_a = props.RelatedObjects) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcObjectDefinition(o); });
        }
        return _this;
    }
    return IfcRelNests;
}(IfcRelDecomposes));
exports.IfcRelNests = IfcRelNests;
var IfcRelProjectsElement = /** @class */ (function (_super) {
    __extends(IfcRelProjectsElement, _super);
    function IfcRelProjectsElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelProjectsElement";
        if (props) {
            _this.RelatedFeatureElement = (props.RelatedFeatureElement) ? new IfcFeatureElementAddition(props.RelatedFeatureElement) : undefined;
        }
        return _this;
    }
    return IfcRelProjectsElement;
}(IfcRelDecomposes));
exports.IfcRelProjectsElement = IfcRelProjectsElement;
var IfcRelReferencedInSpatialStructure = /** @class */ (function (_super) {
    __extends(IfcRelReferencedInSpatialStructure, _super);
    function IfcRelReferencedInSpatialStructure(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelReferencedInSpatialStructure";
        if (props) {
            _this.RelatedElements = (_a = props.RelatedElements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProduct(o); });
        }
        return _this;
    }
    return IfcRelReferencedInSpatialStructure;
}(IfcRelConnects));
exports.IfcRelReferencedInSpatialStructure = IfcRelReferencedInSpatialStructure;
var IfcRelSequence = /** @class */ (function (_super) {
    __extends(IfcRelSequence, _super);
    function IfcRelSequence(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelSequence";
        if (props) {
            _this.RelatingProcess = (props.RelatingProcess) ? new IfcProcess(props.RelatingProcess) : undefined;
            _this.RelatedProcess = (props.RelatedProcess) ? new IfcProcess(props.RelatedProcess) : undefined;
            _this.TimeLag = (props.TimeLag) ? new IfcLagTime(props.TimeLag) : undefined;
        }
        return _this;
    }
    return IfcRelSequence;
}(IfcRelConnects));
exports.IfcRelSequence = IfcRelSequence;
var IfcRelServicesBuildings = /** @class */ (function (_super) {
    __extends(IfcRelServicesBuildings, _super);
    function IfcRelServicesBuildings(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelServicesBuildings";
        if (props) {
            _this.RelatingSystem = (props.RelatingSystem) ? new IfcSystem(props.RelatingSystem) : undefined;
            _this.RelatedBuildings = (_a = props.RelatedBuildings) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcSpatialElement(o); });
        }
        return _this;
    }
    return IfcRelServicesBuildings;
}(IfcRelConnects));
exports.IfcRelServicesBuildings = IfcRelServicesBuildings;
var IfcRelSpaceBoundary = /** @class */ (function (_super) {
    __extends(IfcRelSpaceBoundary, _super);
    function IfcRelSpaceBoundary(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelSpaceBoundary";
        if (props) {
            _this.RelatedBuildingElement = (props.RelatedBuildingElement) ? new IfcElement(props.RelatedBuildingElement) : undefined;
            _this.ConnectionGeometry = (props.ConnectionGeometry) ? new IfcConnectionGeometry(props.ConnectionGeometry) : undefined;
        }
        return _this;
    }
    return IfcRelSpaceBoundary;
}(IfcRelConnects));
exports.IfcRelSpaceBoundary = IfcRelSpaceBoundary;
var IfcRelVoidsElement = /** @class */ (function (_super) {
    __extends(IfcRelVoidsElement, _super);
    function IfcRelVoidsElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelVoidsElement";
        if (props) {
            _this.RelatedOpeningElement = (props.RelatedOpeningElement) ? new IfcFeatureElementSubtraction(props.RelatedOpeningElement) : undefined;
        }
        return _this;
    }
    return IfcRelVoidsElement;
}(IfcRelDecomposes));
exports.IfcRelVoidsElement = IfcRelVoidsElement;
var IfcRepresentation = /** @class */ (function (_super) {
    __extends(IfcRepresentation, _super);
    function IfcRepresentation(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRepresentation";
        if (props) {
            _this.ContextOfItems = (props.ContextOfItems) ? new IfcRepresentationContext(props.ContextOfItems) : undefined;
            _this.Items = (_a = props.Items) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRepresentationItem(o); });
        }
        return _this;
    }
    return IfcRepresentation;
}(Entity));
exports.IfcRepresentation = IfcRepresentation;
var IfcRepresentationItem = /** @class */ (function (_super) {
    __extends(IfcRepresentationItem, _super);
    function IfcRepresentationItem(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRepresentationItem";
        if (props) {
            _this.StyledByItem = (props.StyledByItem) ? new IfcStyledItem(props.StyledByItem) : undefined;
        }
        return _this;
    }
    return IfcRepresentationItem;
}(Entity));
exports.IfcRepresentationItem = IfcRepresentationItem;
var IfcRepresentationMap = /** @class */ (function (_super) {
    __extends(IfcRepresentationMap, _super);
    function IfcRepresentationMap(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRepresentationMap";
        if (props) {
            _this.MappedRepresentation = (props.MappedRepresentation) ? new IfcRepresentation(props.MappedRepresentation) : undefined;
        }
        return _this;
    }
    return IfcRepresentationMap;
}(Entity));
exports.IfcRepresentationMap = IfcRepresentationMap;
var IfcResourceApprovalRelationship = /** @class */ (function (_super) {
    __extends(IfcResourceApprovalRelationship, _super);
    function IfcResourceApprovalRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcResourceApprovalRelationship";
        if (props) {
            _this.RelatingApproval = (props.RelatingApproval) ? new IfcApproval(props.RelatingApproval) : undefined;
        }
        return _this;
    }
    return IfcResourceApprovalRelationship;
}(IfcResourceLevelRelationship));
exports.IfcResourceApprovalRelationship = IfcResourceApprovalRelationship;
var IfcResourceConstraintRelationship = /** @class */ (function (_super) {
    __extends(IfcResourceConstraintRelationship, _super);
    function IfcResourceConstraintRelationship(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcResourceConstraintRelationship";
        if (props) {
            _this.RelatingConstraint = (props.RelatingConstraint) ? new IfcConstraint(props.RelatingConstraint) : undefined;
        }
        return _this;
    }
    return IfcResourceConstraintRelationship;
}(IfcResourceLevelRelationship));
exports.IfcResourceConstraintRelationship = IfcResourceConstraintRelationship;
var IfcRoot = /** @class */ (function (_super) {
    __extends(IfcRoot, _super);
    function IfcRoot(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRoot";
        if (props) {
            _this.OwnerHistory = (props.OwnerHistory) ? new IfcOwnerHistory(props.OwnerHistory) : undefined;
        }
        return _this;
    }
    return IfcRoot;
}(Entity));
exports.IfcRoot = IfcRoot;
var IfcSectionProperties = /** @class */ (function (_super) {
    __extends(IfcSectionProperties, _super);
    function IfcSectionProperties(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSectionProperties";
        if (props) {
            _this.StartProfile = (_a = props.StartProfile) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
            _this.EndProfile = (_b = props.EndProfile) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcSectionProperties;
}(IfcPreDefinedProperties));
exports.IfcSectionProperties = IfcSectionProperties;
var IfcSectionReinforcementProperties = /** @class */ (function (_super) {
    __extends(IfcSectionReinforcementProperties, _super);
    function IfcSectionReinforcementProperties(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSectionReinforcementProperties";
        if (props) {
            _this.SectionDefinition = (props.SectionDefinition) ? new IfcSectionProperties(props.SectionDefinition) : undefined;
            _this.CrossSectionReinforcementDefinitions = (_a = props.CrossSectionReinforcementDefinitions) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcReinforcementBarProperties(o); });
        }
        return _this;
    }
    return IfcSectionReinforcementProperties;
}(IfcPreDefinedProperties));
exports.IfcSectionReinforcementProperties = IfcSectionReinforcementProperties;
var IfcSectionedSpine = /** @class */ (function (_super) {
    __extends(IfcSectionedSpine, _super);
    function IfcSectionedSpine(props) {
        var _a, _b, _c;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSectionedSpine";
        if (props) {
            _this.SpineCurve = (_a = props.SpineCurve) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCompositeCurveSegment(o); });
            _this.CrossSections = (_b = props.CrossSections) === null || _b === void 0 ? void 0 : _b.map(function (o) { return o; });
            _this.CrossSectionPositions = (_c = props.CrossSectionPositions) === null || _c === void 0 ? void 0 : _c.map(function (o) { return new IfcAxis2Placement3D(o); });
        }
        return _this;
    }
    return IfcSectionedSpine;
}(IfcGeometricRepresentationItem));
exports.IfcSectionedSpine = IfcSectionedSpine;
var IfcShellBasedSurfaceModel = /** @class */ (function (_super) {
    __extends(IfcShellBasedSurfaceModel, _super);
    function IfcShellBasedSurfaceModel(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcShellBasedSurfaceModel";
        return _this;
    }
    return IfcShellBasedSurfaceModel;
}(IfcGeometricRepresentationItem));
exports.IfcShellBasedSurfaceModel = IfcShellBasedSurfaceModel;
var IfcSimplePropertyTemplate = /** @class */ (function (_super) {
    __extends(IfcSimplePropertyTemplate, _super);
    function IfcSimplePropertyTemplate(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSimplePropertyTemplate";
        if (props) {
            _this.Enumerators = (props.Enumerators) ? new IfcPropertyEnumeration(props.Enumerators) : undefined;
        }
        return _this;
    }
    return IfcSimplePropertyTemplate;
}(IfcPropertyTemplate));
exports.IfcSimplePropertyTemplate = IfcSimplePropertyTemplate;
var IfcSite = /** @class */ (function (_super) {
    __extends(IfcSite, _super);
    function IfcSite() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcSite;
}(IfcSpatialStructureElement));
exports.IfcSite = IfcSite;
-wrapper[];
constructor(props ?  : IfcSite);
{
    _this = _super.call(this, props) || this;
    this["@class"] = ".IfcSite";
    if (props) {
        this.SiteAddress = (_e = props.SiteAddress) === null || _e === void 0 ? void 0 : _e.map(function (o) { return new IfcLabel - wrapper(o); });
    }
}
var IfcStructuralAnalysisModel = /** @class */ (function (_super) {
    __extends(IfcStructuralAnalysisModel, _super);
    function IfcStructuralAnalysisModel(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralAnalysisModel";
        if (props) {
            _this.OrientationOf2DPlane = (props.OrientationOf2DPlane) ? new IfcAxis2Placement3D(props.OrientationOf2DPlane) : undefined;
            _this.LoadedBy = (_a = props.LoadedBy) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcStructuralLoadGroup(o); });
            _this.HasResults = (_b = props.HasResults) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcStructuralResultGroup(o); });
            _this.SharedPlacement = (props.SharedPlacement) ? new IfcObjectPlacement(props.SharedPlacement) : undefined;
        }
        return _this;
    }
    return IfcStructuralAnalysisModel;
}(IfcSystem));
exports.IfcStructuralAnalysisModel = IfcStructuralAnalysisModel;
var IfcStructuralConnection = /** @class */ (function (_super) {
    __extends(IfcStructuralConnection, _super);
    function IfcStructuralConnection(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralConnection";
        if (props) {
            _this.AppliedCondition = (props.AppliedCondition) ? new IfcBoundaryCondition(props.AppliedCondition) : undefined;
        }
        return _this;
    }
    return IfcStructuralConnection;
}(IfcStructuralItem));
exports.IfcStructuralConnection = IfcStructuralConnection;
var IfcStructuralCurveMember = /** @class */ (function (_super) {
    __extends(IfcStructuralCurveMember, _super);
    function IfcStructuralCurveMember(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralCurveMember";
        if (props) {
            _this.Axis = (props.Axis) ? new IfcDirection(props.Axis) : undefined;
        }
        return _this;
    }
    return IfcStructuralCurveMember;
}(IfcStructuralMember));
exports.IfcStructuralCurveMember = IfcStructuralCurveMember;
var IfcStructuralLoadConfiguration = /** @class */ (function (_super) {
    __extends(IfcStructuralLoadConfiguration, _super);
    function IfcStructuralLoadConfiguration() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcStructuralLoadConfiguration;
}(IfcStructuralLoad));
exports.IfcStructuralLoadConfiguration = IfcStructuralLoadConfiguration;
-wrapper[];
constructor(props ?  : IfcStructuralLoadConfiguration);
{
    _this = _super.call(this, props) || this;
    this["@class"] = ".IfcStructuralLoadConfiguration";
    if (props) {
        this.Values = (_f = props.Values) === null || _f === void 0 ? void 0 : _f.map(function (o) { return new IfcIrregularTimeSeriesValue(o); });
        this.Locations = (_g = props.Locations) === null || _g === void 0 ? void 0 : _g.map(function (o) { return new IfcLengthMeasure - wrapper(o); });
    }
}
var IfcStructuralResultGroup = /** @class */ (function (_super) {
    __extends(IfcStructuralResultGroup, _super);
    function IfcStructuralResultGroup(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralResultGroup";
        if (props) {
            _this.ResultForLoadGroup = (props.ResultForLoadGroup) ? new IfcStructuralLoadGroup(props.ResultForLoadGroup) : undefined;
        }
        return _this;
    }
    return IfcStructuralResultGroup;
}(IfcGroup));
exports.IfcStructuralResultGroup = IfcStructuralResultGroup;
var IfcSurfaceCurve = /** @class */ (function (_super) {
    __extends(IfcSurfaceCurve, _super);
    function IfcSurfaceCurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceCurve";
        if (props) {
            _this.Curve3D = (props.Curve3D) ? new IfcCurve(props.Curve3D) : undefined;
            _this.AssociatedGeometry = (props.AssociatedGeometry) ? new AssociatedGeometry(props.AssociatedGeometry) : undefined;
        }
        return _this;
    }
    return IfcSurfaceCurve;
}(IfcCurve));
exports.IfcSurfaceCurve = IfcSurfaceCurve;
var IfcSurfaceStyle = /** @class */ (function (_super) {
    __extends(IfcSurfaceStyle, _super);
    function IfcSurfaceStyle(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceStyle";
        return _this;
    }
    return IfcSurfaceStyle;
}(IfcPresentationStyle));
exports.IfcSurfaceStyle = IfcSurfaceStyle;
var IfcSurfaceStyleLighting = /** @class */ (function (_super) {
    __extends(IfcSurfaceStyleLighting, _super);
    function IfcSurfaceStyleLighting(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceStyleLighting";
        if (props) {
            _this.DiffuseTransmissionColour = (props.DiffuseTransmissionColour) ? new IfcColourRgb(props.DiffuseTransmissionColour) : undefined;
            _this.DiffuseReflectionColour = (props.DiffuseReflectionColour) ? new IfcColourRgb(props.DiffuseReflectionColour) : undefined;
            _this.TransmissionColour = (props.TransmissionColour) ? new IfcColourRgb(props.TransmissionColour) : undefined;
            _this.ReflectanceColour = (props.ReflectanceColour) ? new IfcColourRgb(props.ReflectanceColour) : undefined;
        }
        return _this;
    }
    return IfcSurfaceStyleLighting;
}(IfcPresentationItem));
exports.IfcSurfaceStyleLighting = IfcSurfaceStyleLighting;
var IfcSurfaceStyleShading = /** @class */ (function (_super) {
    __extends(IfcSurfaceStyleShading, _super);
    function IfcSurfaceStyleShading(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceStyleShading";
        if (props) {
            _this.SurfaceColour = (props.SurfaceColour) ? new IfcColourRgb(props.SurfaceColour) : undefined;
        }
        return _this;
    }
    return IfcSurfaceStyleShading;
}(IfcPresentationItem));
exports.IfcSurfaceStyleShading = IfcSurfaceStyleShading;
var IfcSurfaceStyleWithTextures = /** @class */ (function (_super) {
    __extends(IfcSurfaceStyleWithTextures, _super);
    function IfcSurfaceStyleWithTextures(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceStyleWithTextures";
        if (props) {
            _this.Textures = (_a = props.Textures) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcSurfaceTexture(o); });
        }
        return _this;
    }
    return IfcSurfaceStyleWithTextures;
}(IfcPresentationItem));
exports.IfcSurfaceStyleWithTextures = IfcSurfaceStyleWithTextures;
var IfcSurfaceTexture = /** @class */ (function (_super) {
    __extends(IfcSurfaceTexture, _super);
    function IfcSurfaceTexture(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceTexture";
        if (props) {
            _this.TextureTransform = (props.TextureTransform) ? new IfcCartesianTransformationOperator2D(props.TextureTransform) : undefined;
        }
        return _this;
    }
    return IfcSurfaceTexture;
}(IfcPresentationItem));
exports.IfcSurfaceTexture = IfcSurfaceTexture;
var IfcSweptAreaSolid = /** @class */ (function (_super) {
    __extends(IfcSweptAreaSolid, _super);
    function IfcSweptAreaSolid(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSweptAreaSolid";
        if (props) {
            _this.SweptArea = (_a = props.SweptArea) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
        }
        return _this;
    }
    return IfcSweptAreaSolid;
}(IfcSolidModel));
exports.IfcSweptAreaSolid = IfcSweptAreaSolid;
var IfcSweptDiskSolid = /** @class */ (function (_super) {
    __extends(IfcSweptDiskSolid, _super);
    function IfcSweptDiskSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSweptDiskSolid";
        if (props) {
            _this.Directrix = (props.Directrix) ? new IfcCurve(props.Directrix) : undefined;
        }
        return _this;
    }
    return IfcSweptDiskSolid;
}(IfcSolidModel));
exports.IfcSweptDiskSolid = IfcSweptDiskSolid;
var IfcSweptSurface = /** @class */ (function (_super) {
    __extends(IfcSweptSurface, _super);
    function IfcSweptSurface(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSweptSurface";
        if (props) {
            _this.SweptCurve = (_a = props.SweptCurve) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
        }
        return _this;
    }
    return IfcSweptSurface;
}(IfcSurface));
exports.IfcSweptSurface = IfcSweptSurface;
var IfcTable = /** @class */ (function (_super) {
    __extends(IfcTable, _super);
    function IfcTable(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTable";
        if (props) {
            _this.Rows = (_a = props.Rows) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcTableRow(o); });
            _this.Columns = (_b = props.Columns) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcTableColumn(o); });
        }
        return _this;
    }
    return IfcTable;
}(Entity));
exports.IfcTable = IfcTable;
var IfcTableColumn = /** @class */ (function (_super) {
    __extends(IfcTableColumn, _super);
    function IfcTableColumn(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTableColumn";
        if (props) {
            _this.ReferencePath = (props.ReferencePath) ? new IfcReference(props.ReferencePath) : undefined;
        }
        return _this;
    }
    return IfcTableColumn;
}(Entity));
exports.IfcTableColumn = IfcTableColumn;
var IfcTableRow = /** @class */ (function (_super) {
    __extends(IfcTableRow, _super);
    function IfcTableRow(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTableRow";
        return _this;
    }
    return IfcTableRow;
}(Entity));
exports.IfcTableRow = IfcTableRow;
var IfcTask = /** @class */ (function (_super) {
    __extends(IfcTask, _super);
    function IfcTask(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTask";
        if (props) {
            _this.TaskTime = (props.TaskTime) ? new IfcTaskTime(props.TaskTime) : undefined;
        }
        return _this;
    }
    return IfcTask;
}(IfcProcess));
exports.IfcTask = IfcTask;
var IfcTaskTimeRecurring = /** @class */ (function (_super) {
    __extends(IfcTaskTimeRecurring, _super);
    function IfcTaskTimeRecurring(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTaskTimeRecurring";
        if (props) {
            _this.Recurrence = (_a = props.Recurrence) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcTimePeriod(o); });
        }
        return _this;
    }
    return IfcTaskTimeRecurring;
}(IfcTaskTime));
exports.IfcTaskTimeRecurring = IfcTaskTimeRecurring;
var IfcTessellatedFaceSet = /** @class */ (function (_super) {
    __extends(IfcTessellatedFaceSet, _super);
    function IfcTessellatedFaceSet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTessellatedFaceSet";
        if (props) {
            _this.Coordinates = (props.Coordinates) ? new IfcCartesianPointList3D(props.Coordinates) : undefined;
            _this.HasColours = (props.HasColours) ? new IfcIndexedColourMap(props.HasColours) : undefined;
            _this.HasTextures = (_a = props.HasTextures) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcIndexedTextureMap(o); });
        }
        return _this;
    }
    return IfcTessellatedFaceSet;
}(IfcTessellatedItem));
exports.IfcTessellatedFaceSet = IfcTessellatedFaceSet;
var IfcTextLiteral = /** @class */ (function (_super) {
    __extends(IfcTextLiteral, _super);
    function IfcTextLiteral(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextLiteral";
        return _this;
    }
    return IfcTextLiteral;
}(IfcGeometricRepresentationItem));
exports.IfcTextLiteral = IfcTextLiteral;
var IfcTextStyle = /** @class */ (function (_super) {
    __extends(IfcTextStyle, _super);
    function IfcTextStyle(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextStyle";
        if (props) {
            _this.TextCharacterAppearance = (props.TextCharacterAppearance) ? new IfcTextStyleForDefinedFont(props.TextCharacterAppearance) : undefined;
            _this.TextStyle = (props.TextStyle) ? new IfcTextStyleTextModel(props.TextStyle) : undefined;
        }
        return _this;
    }
    return IfcTextStyle;
}(IfcPresentationStyle));
exports.IfcTextStyle = IfcTextStyle;
var IfcTextStyleFontModel = /** @class */ (function (_super) {
    __extends(IfcTextStyleFontModel, _super);
    function IfcTextStyleFontModel() {
        return _super !== null && _super.apply(this, arguments) || this;
    }
    return IfcTextStyleFontModel;
}(IfcPreDefinedTextFont));
exports.IfcTextStyleFontModel = IfcTextStyleFontModel;
-wrapper[];
constructor(props ?  : IfcTextStyleFontModel);
{
    _this = _super.call(this, props) || this;
    this["@class"] = ".IfcTextStyleFontModel";
    if (props) {
        this.FontFamily = (_h = props.FontFamily) === null || _h === void 0 ? void 0 : _h.map(function (o) { return new IfcTextFontName - wrapper(o); });
    }
}
var IfcTextStyleForDefinedFont = /** @class */ (function (_super) {
    __extends(IfcTextStyleForDefinedFont, _super);
    function IfcTextStyleForDefinedFont(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextStyleForDefinedFont";
        return _this;
    }
    return IfcTextStyleForDefinedFont;
}(IfcPresentationItem));
exports.IfcTextStyleForDefinedFont = IfcTextStyleForDefinedFont;
var IfcTextStyleTextModel = /** @class */ (function (_super) {
    __extends(IfcTextStyleTextModel, _super);
    function IfcTextStyleTextModel(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextStyleTextModel";
        return _this;
    }
    return IfcTextStyleTextModel;
}(IfcPresentationItem));
exports.IfcTextStyleTextModel = IfcTextStyleTextModel;
var IfcTextureCoordinate = /** @class */ (function (_super) {
    __extends(IfcTextureCoordinate, _super);
    function IfcTextureCoordinate(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextureCoordinate";
        if (props) {
            _this.Maps = (_a = props.Maps) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcSurfaceTexture(o); });
        }
        return _this;
    }
    return IfcTextureCoordinate;
}(IfcPresentationItem));
exports.IfcTextureCoordinate = IfcTextureCoordinate;
var IfcTimeSeries = /** @class */ (function (_super) {
    __extends(IfcTimeSeries, _super);
    function IfcTimeSeries(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTimeSeries";
        return _this;
    }
    return IfcTimeSeries;
}(Entity));
exports.IfcTimeSeries = IfcTimeSeries;
var IfcTimeSeriesValue = /** @class */ (function (_super) {
    __extends(IfcTimeSeriesValue, _super);
    function IfcTimeSeriesValue(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTimeSeriesValue";
        return _this;
    }
    return IfcTimeSeriesValue;
}(Entity));
exports.IfcTimeSeriesValue = IfcTimeSeriesValue;
var IfcTrimmedCurve = /** @class */ (function (_super) {
    __extends(IfcTrimmedCurve, _super);
    function IfcTrimmedCurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTrimmedCurve";
        if (props) {
            _this.BasisCurve = (props.BasisCurve) ? new IfcCurve(props.BasisCurve) : undefined;
        }
        return _this;
    }
    return IfcTrimmedCurve;
}(IfcBoundedCurve));
exports.IfcTrimmedCurve = IfcTrimmedCurve;
var IfcUnitAssignment = /** @class */ (function (_super) {
    __extends(IfcUnitAssignment, _super);
    function IfcUnitAssignment(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcUnitAssignment";
        return _this;
    }
    return IfcUnitAssignment;
}(Entity));
exports.IfcUnitAssignment = IfcUnitAssignment;
var IfcVector = /** @class */ (function (_super) {
    __extends(IfcVector, _super);
    function IfcVector(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcVector";
        if (props) {
            _this.Orientation = (props.Orientation) ? new IfcDirection(props.Orientation) : undefined;
        }
        return _this;
    }
    return IfcVector;
}(IfcGeometricRepresentationItem));
exports.IfcVector = IfcVector;
var IfcVertexLoop = /** @class */ (function (_super) {
    __extends(IfcVertexLoop, _super);
    function IfcVertexLoop(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcVertexLoop";
        if (props) {
            _this.LoopVertex = (props.LoopVertex) ? new IfcVertex(props.LoopVertex) : undefined;
        }
        return _this;
    }
    return IfcVertexLoop;
}(IfcLoop));
exports.IfcVertexLoop = IfcVertexLoop;
var IfcVertexPoint = /** @class */ (function (_super) {
    __extends(IfcVertexPoint, _super);
    function IfcVertexPoint(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcVertexPoint";
        if (props) {
            _this.VertexGeometry = (props.VertexGeometry) ? new IfcPoint(props.VertexGeometry) : undefined;
        }
        return _this;
    }
    return IfcVertexPoint;
}(IfcVertex));
exports.IfcVertexPoint = IfcVertexPoint;
var IfcVirtualGridIntersection = /** @class */ (function (_super) {
    __extends(IfcVirtualGridIntersection, _super);
    function IfcVirtualGridIntersection(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcVirtualGridIntersection";
        if (props) {
            _this.IntersectingAxes = (props.IntersectingAxes) ? new IntersectingAxes(props.IntersectingAxes) : undefined;
        }
        return _this;
    }
    return IfcVirtualGridIntersection;
}(Entity));
exports.IfcVirtualGridIntersection = IfcVirtualGridIntersection;
var IfcWindowLiningProperties = /** @class */ (function (_super) {
    __extends(IfcWindowLiningProperties, _super);
    function IfcWindowLiningProperties(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcWindowLiningProperties";
        if (props) {
            _this.ShapeAspectStyle = (_a = props.ShapeAspectStyle) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcShapeModel(o); });
        }
        return _this;
    }
    return IfcWindowLiningProperties;
}(IfcPreDefinedPropertySet));
exports.IfcWindowLiningProperties = IfcWindowLiningProperties;
var IfcWindowPanelProperties = /** @class */ (function (_super) {
    __extends(IfcWindowPanelProperties, _super);
    function IfcWindowPanelProperties(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcWindowPanelProperties";
        if (props) {
            _this.ShapeAspectStyle = (_a = props.ShapeAspectStyle) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcShapeModel(o); });
        }
        return _this;
    }
    return IfcWindowPanelProperties;
}(IfcPreDefinedPropertySet));
exports.IfcWindowPanelProperties = IfcWindowPanelProperties;
var IfcWorkCalendar = /** @class */ (function (_super) {
    __extends(IfcWorkCalendar, _super);
    function IfcWorkCalendar(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcWorkCalendar";
        if (props) {
            _this.WorkingTimes = (_a = props.WorkingTimes) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcWorkTime(o); });
            _this.ExceptionTimes = (_b = props.ExceptionTimes) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcWorkTime(o); });
        }
        return _this;
    }
    return IfcWorkCalendar;
}(IfcControl));
exports.IfcWorkCalendar = IfcWorkCalendar;
var IfcWorkControl = /** @class */ (function (_super) {
    __extends(IfcWorkControl, _super);
    function IfcWorkControl(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcWorkControl";
        if (props) {
            _this.Creators = (_a = props.Creators) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPerson(o); });
        }
        return _this;
    }
    return IfcWorkControl;
}(IfcControl));
exports.IfcWorkControl = IfcWorkControl;
var IfcWorkTime = /** @class */ (function (_super) {
    __extends(IfcWorkTime, _super);
    function IfcWorkTime(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcWorkTime";
        if (props) {
            _this.RecurrencePattern = (_a = props.RecurrencePattern) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcTimePeriod(o); });
        }
        return _this;
    }
    return IfcWorkTime;
}(IfcSchedulingTime));
exports.IfcWorkTime = IfcWorkTime;
var IfcArbitraryClosedProfileDef = /** @class */ (function (_super) {
    __extends(IfcArbitraryClosedProfileDef, _super);
    function IfcArbitraryClosedProfileDef(props) {
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcArbitraryClosedProfileDef";
        if (props) {
            _this.OuterCurve = (props.OuterCurve) ? new IfcCurve(props.OuterCurve) : undefined;
        }
        return _this;
    }
    return IfcArbitraryClosedProfileDef;
}(IfcProfileDef));
exports.IfcArbitraryClosedProfileDef = IfcArbitraryClosedProfileDef;
var IfcArbitraryOpenProfileDef = /** @class */ (function (_super) {
    __extends(IfcArbitraryOpenProfileDef, _super);
    function IfcArbitraryOpenProfileDef(props) {
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcArbitraryOpenProfileDef";
        if (props) {
            _this.Curve = (props.Curve) ? new IfcBoundedCurve(props.Curve) : undefined;
        }
        return _this;
    }
    return IfcArbitraryOpenProfileDef;
}(IfcProfileDef));
exports.IfcArbitraryOpenProfileDef = IfcArbitraryOpenProfileDef;
var IfcAxis1Placement = /** @class */ (function (_super) {
    __extends(IfcAxis1Placement, _super);
    function IfcAxis1Placement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAxis1Placement";
        if (props) {
            _this.Axis = (props.Axis) ? new IfcDirection(props.Axis) : undefined;
        }
        return _this;
    }
    return IfcAxis1Placement;
}(IfcPlacement));
exports.IfcAxis1Placement = IfcAxis1Placement;
var IfcAxis2Placement2D = /** @class */ (function (_super) {
    __extends(IfcAxis2Placement2D, _super);
    function IfcAxis2Placement2D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAxis2Placement2D";
        if (props) {
            _this.RefDirection = (props.RefDirection) ? new IfcDirection(props.RefDirection) : undefined;
        }
        return _this;
    }
    return IfcAxis2Placement2D;
}(IfcPlacement));
exports.IfcAxis2Placement2D = IfcAxis2Placement2D;
var IfcAxis2Placement3D = /** @class */ (function (_super) {
    __extends(IfcAxis2Placement3D, _super);
    function IfcAxis2Placement3D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcAxis2Placement3D";
        if (props) {
            _this.Axis = (props.Axis) ? new IfcDirection(props.Axis) : undefined;
            _this.RefDirection = (props.RefDirection) ? new IfcDirection(props.RefDirection) : undefined;
        }
        return _this;
    }
    return IfcAxis2Placement3D;
}(IfcPlacement));
exports.IfcAxis2Placement3D = IfcAxis2Placement3D;
var IfcBlobTexture = /** @class */ (function (_super) {
    __extends(IfcBlobTexture, _super);
    function IfcBlobTexture(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBlobTexture";
        if (props) {
            _this.RasterCode = (props.RasterCode) ? new IfcBinary(props.RasterCode) : undefined;
        }
        return _this;
    }
    return IfcBlobTexture;
}(IfcSurfaceTexture));
exports.IfcBlobTexture = IfcBlobTexture;
var IfcBoundaryNodeConditionWarping = /** @class */ (function (_super) {
    __extends(IfcBoundaryNodeConditionWarping, _super);
    function IfcBoundaryNodeConditionWarping(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoundaryNodeConditionWarping";
        return _this;
    }
    return IfcBoundaryNodeConditionWarping;
}(IfcBoundaryNodeCondition));
exports.IfcBoundaryNodeConditionWarping = IfcBoundaryNodeConditionWarping;
var IfcBoxedHalfSpace = /** @class */ (function (_super) {
    __extends(IfcBoxedHalfSpace, _super);
    function IfcBoxedHalfSpace(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcBoxedHalfSpace";
        if (props) {
            _this.Enclosure = (props.Enclosure) ? new IfcBoundingBox(props.Enclosure) : undefined;
        }
        return _this;
    }
    return IfcBoxedHalfSpace;
}(IfcHalfSpaceSolid));
exports.IfcBoxedHalfSpace = IfcBoxedHalfSpace;
var IfcCartesianTransformationOperator3D = /** @class */ (function (_super) {
    __extends(IfcCartesianTransformationOperator3D, _super);
    function IfcCartesianTransformationOperator3D(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcCartesianTransformationOperator3D";
        if (props) {
            _this.Axis3 = (props.Axis3) ? new IfcDirection(props.Axis3) : undefined;
        }
        return _this;
    }
    return IfcCartesianTransformationOperator3D;
}(IfcCartesianTransformationOperator));
exports.IfcCartesianTransformationOperator3D = IfcCartesianTransformationOperator3D;
var IfcConversionBasedUnit = /** @class */ (function (_super) {
    __extends(IfcConversionBasedUnit, _super);
    function IfcConversionBasedUnit(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcConversionBasedUnit";
        if (props) {
            _this.ConversionFactor = (props.ConversionFactor) ? new IfcMeasureWithUnit(props.ConversionFactor) : undefined;
        }
        return _this;
    }
    return IfcConversionBasedUnit;
}(IfcNamedUnit));
exports.IfcConversionBasedUnit = IfcConversionBasedUnit;
var IfcDerivedProfileDef = /** @class */ (function (_super) {
    __extends(IfcDerivedProfileDef, _super);
    function IfcDerivedProfileDef(props) {
        var _a;
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcDerivedProfileDef";
        if (props) {
            _this.ParentProfile = (_a = props.ParentProfile) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
            _this.Operator = (props.Operator) ? new IfcCartesianTransformationOperator2D(props.Operator) : undefined;
        }
        return _this;
    }
    return IfcDerivedProfileDef;
}(IfcProfileDef));
exports.IfcDerivedProfileDef = IfcDerivedProfileDef;
var IfcEdgeCurve = /** @class */ (function (_super) {
    __extends(IfcEdgeCurve, _super);
    function IfcEdgeCurve(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcEdgeCurve";
        if (props) {
            _this.EdgeGeometry = (props.EdgeGeometry) ? new IfcCurve(props.EdgeGeometry) : undefined;
        }
        return _this;
    }
    return IfcEdgeCurve;
}(IfcEdge));
exports.IfcEdgeCurve = IfcEdgeCurve;
var IfcExtrudedAreaSolid = /** @class */ (function (_super) {
    __extends(IfcExtrudedAreaSolid, _super);
    function IfcExtrudedAreaSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcExtrudedAreaSolid";
        if (props) {
            _this.ExtrudedDirection = (props.ExtrudedDirection) ? new IfcDirection(props.ExtrudedDirection) : undefined;
        }
        return _this;
    }
    return IfcExtrudedAreaSolid;
}(IfcSweptAreaSolid));
exports.IfcExtrudedAreaSolid = IfcExtrudedAreaSolid;
var IfcFaceSurface = /** @class */ (function (_super) {
    __extends(IfcFaceSurface, _super);
    function IfcFaceSurface(props) {
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcFaceSurface";
        if (props) {
            _this.FaceSurface = (props.FaceSurface) ? new IfcSurface(props.FaceSurface) : undefined;
        }
        return _this;
    }
    return IfcFaceSurface;
}(IfcFace));
exports.IfcFaceSurface = IfcFaceSurface;
var IfcFixedReferenceSweptAreaSolid = /** @class */ (function (_super) {
    __extends(IfcFixedReferenceSweptAreaSolid, _super);
    function IfcFixedReferenceSweptAreaSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcFixedReferenceSweptAreaSolid";
        if (props) {
            _this.Directrix = (props.Directrix) ? new IfcCurve(props.Directrix) : undefined;
            _this.FixedReference = (props.FixedReference) ? new IfcDirection(props.FixedReference) : undefined;
        }
        return _this;
    }
    return IfcFixedReferenceSweptAreaSolid;
}(IfcSweptAreaSolid));
exports.IfcFixedReferenceSweptAreaSolid = IfcFixedReferenceSweptAreaSolid;
var IfcIndexedTextureMap = /** @class */ (function (_super) {
    __extends(IfcIndexedTextureMap, _super);
    function IfcIndexedTextureMap(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcIndexedTextureMap";
        if (props) {
            _this.TexCoords = (props.TexCoords) ? new IfcTextureVertexList(props.TexCoords) : undefined;
        }
        return _this;
    }
    return IfcIndexedTextureMap;
}(IfcTextureCoordinate));
exports.IfcIndexedTextureMap = IfcIndexedTextureMap;
var IfcIrregularTimeSeries = /** @class */ (function (_super) {
    __extends(IfcIrregularTimeSeries, _super);
    function IfcIrregularTimeSeries(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcIrregularTimeSeries";
        if (props) {
            _this.Values = (_a = props.Values) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcIrregularTimeSeriesValue(o); });
        }
        return _this;
    }
    return IfcIrregularTimeSeries;
}(IfcTimeSeries));
exports.IfcIrregularTimeSeries = IfcIrregularTimeSeries;
var IfcLightSourceDirectional = /** @class */ (function (_super) {
    __extends(IfcLightSourceDirectional, _super);
    function IfcLightSourceDirectional(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightSourceDirectional";
        if (props) {
            _this.Orientation = (props.Orientation) ? new IfcDirection(props.Orientation) : undefined;
        }
        return _this;
    }
    return IfcLightSourceDirectional;
}(IfcLightSource));
exports.IfcLightSourceDirectional = IfcLightSourceDirectional;
var IfcLightSourceGoniometric = /** @class */ (function (_super) {
    __extends(IfcLightSourceGoniometric, _super);
    function IfcLightSourceGoniometric(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightSourceGoniometric";
        if (props) {
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
            _this.ColourAppearance = (props.ColourAppearance) ? new IfcColourRgb(props.ColourAppearance) : undefined;
        }
        return _this;
    }
    return IfcLightSourceGoniometric;
}(IfcLightSource));
exports.IfcLightSourceGoniometric = IfcLightSourceGoniometric;
var IfcLightSourcePositional = /** @class */ (function (_super) {
    __extends(IfcLightSourcePositional, _super);
    function IfcLightSourcePositional(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightSourcePositional";
        if (props) {
            _this.Position = (props.Position) ? new IfcCartesianPoint(props.Position) : undefined;
        }
        return _this;
    }
    return IfcLightSourcePositional;
}(IfcLightSource));
exports.IfcLightSourcePositional = IfcLightSourcePositional;
var IfcMappedItem = /** @class */ (function (_super) {
    __extends(IfcMappedItem, _super);
    function IfcMappedItem(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMappedItem";
        if (props) {
            _this.MappingSource = (props.MappingSource) ? new IfcRepresentationMap(props.MappingSource) : undefined;
            _this.MappingTarget = (props.MappingTarget) ? new IfcCartesianTransformationOperator(props.MappingTarget) : undefined;
        }
        return _this;
    }
    return IfcMappedItem;
}(IfcRepresentationItem));
exports.IfcMappedItem = IfcMappedItem;
var IfcMaterial = /** @class */ (function (_super) {
    __extends(IfcMaterial, _super);
    function IfcMaterial(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterial";
        if (props) {
            _this.HasRepresentation = (props.HasRepresentation) ? new IfcMaterialDefinitionRepresentation(props.HasRepresentation) : undefined;
        }
        return _this;
    }
    return IfcMaterial;
}(IfcMaterialDefinition));
exports.IfcMaterial = IfcMaterial;
var IfcMaterialConstituent = /** @class */ (function (_super) {
    __extends(IfcMaterialConstituent, _super);
    function IfcMaterialConstituent(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialConstituent";
        if (props) {
            _this.Material = (props.Material) ? new IfcMaterial(props.Material) : undefined;
        }
        return _this;
    }
    return IfcMaterialConstituent;
}(IfcMaterialDefinition));
exports.IfcMaterialConstituent = IfcMaterialConstituent;
var IfcMaterialConstituentSet = /** @class */ (function (_super) {
    __extends(IfcMaterialConstituentSet, _super);
    function IfcMaterialConstituentSet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialConstituentSet";
        if (props) {
            _this.MaterialConstituents = (_a = props.MaterialConstituents) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcMaterialConstituent(o); });
        }
        return _this;
    }
    return IfcMaterialConstituentSet;
}(IfcMaterialDefinition));
exports.IfcMaterialConstituentSet = IfcMaterialConstituentSet;
var IfcMaterialLayer = /** @class */ (function (_super) {
    __extends(IfcMaterialLayer, _super);
    function IfcMaterialLayer(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialLayer";
        if (props) {
            _this.Material = (props.Material) ? new IfcMaterial(props.Material) : undefined;
        }
        return _this;
    }
    return IfcMaterialLayer;
}(IfcMaterialDefinition));
exports.IfcMaterialLayer = IfcMaterialLayer;
var IfcMaterialLayerSet = /** @class */ (function (_super) {
    __extends(IfcMaterialLayerSet, _super);
    function IfcMaterialLayerSet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialLayerSet";
        if (props) {
            _this.MaterialLayers = (_a = props.MaterialLayers) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcMaterialLayer(o); });
        }
        return _this;
    }
    return IfcMaterialLayerSet;
}(IfcMaterialDefinition));
exports.IfcMaterialLayerSet = IfcMaterialLayerSet;
var IfcMaterialProfile = /** @class */ (function (_super) {
    __extends(IfcMaterialProfile, _super);
    function IfcMaterialProfile(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialProfile";
        if (props) {
            _this.Material = (props.Material) ? new IfcMaterial(props.Material) : undefined;
            _this.Profile = (_a = props.Profile) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcMaterialProfile;
}(IfcMaterialDefinition));
exports.IfcMaterialProfile = IfcMaterialProfile;
var IfcMaterialProfileSet = /** @class */ (function (_super) {
    __extends(IfcMaterialProfileSet, _super);
    function IfcMaterialProfileSet(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialProfileSet";
        if (props) {
            _this.MaterialProfiles = (_a = props.MaterialProfiles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcMaterialProfile(o); });
            _this.CompositeProfile = (_b = props.CompositeProfile) === null || _b === void 0 ? void 0 : _b.map(function (o) { return o; });
        }
        return _this;
    }
    return IfcMaterialProfileSet;
}(IfcMaterialDefinition));
exports.IfcMaterialProfileSet = IfcMaterialProfileSet;
var IfcMaterialProfileSetUsageTapering = /** @class */ (function (_super) {
    __extends(IfcMaterialProfileSetUsageTapering, _super);
    function IfcMaterialProfileSetUsageTapering(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMaterialProfileSetUsageTapering";
        if (props) {
            _this.ForProfileEndSet = (props.ForProfileEndSet) ? new IfcMaterialProfileSet(props.ForProfileEndSet) : undefined;
        }
        return _this;
    }
    return IfcMaterialProfileSetUsageTapering;
}(IfcMaterialProfileSetUsage));
exports.IfcMaterialProfileSetUsageTapering = IfcMaterialProfileSetUsageTapering;
var IfcMetric = /** @class */ (function (_super) {
    __extends(IfcMetric, _super);
    function IfcMetric(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcMetric";
        if (props) {
            _this.ReferencePath = (props.ReferencePath) ? new IfcReference(props.ReferencePath) : undefined;
        }
        return _this;
    }
    return IfcMetric;
}(IfcConstraint));
exports.IfcMetric = IfcMetric;
var IfcObjectDefinition = /** @class */ (function (_super) {
    __extends(IfcObjectDefinition, _super);
    function IfcObjectDefinition(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcObjectDefinition";
        if (props) {
            _this.IsNestedBy = (_a = props.IsNestedBy) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRelNests(o); });
            _this.IsDecomposedBy = (_b = props.IsDecomposedBy) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcRelAggregates(o); });
        }
        return _this;
    }
    return IfcObjectDefinition;
}(IfcRoot));
exports.IfcObjectDefinition = IfcObjectDefinition;
var IfcObjective = /** @class */ (function (_super) {
    __extends(IfcObjective, _super);
    function IfcObjective(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcObjective";
        if (props) {
            _this.BenchmarkValues = (_a = props.BenchmarkValues) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcConstraint(o); });
        }
        return _this;
    }
    return IfcObjective;
}(IfcConstraint));
exports.IfcObjective = IfcObjective;
var IfcParameterizedProfileDef = /** @class */ (function (_super) {
    __extends(IfcParameterizedProfileDef, _super);
    function IfcParameterizedProfileDef(props) {
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcParameterizedProfileDef";
        if (props) {
            _this.Position = (props.Position) ? new IfcAxis2Placement2D(props.Position) : undefined;
        }
        return _this;
    }
    return IfcParameterizedProfileDef;
}(IfcProfileDef));
exports.IfcParameterizedProfileDef = IfcParameterizedProfileDef;
var IfcPolygonalBoundedHalfSpace = /** @class */ (function (_super) {
    __extends(IfcPolygonalBoundedHalfSpace, _super);
    function IfcPolygonalBoundedHalfSpace(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPolygonalBoundedHalfSpace";
        if (props) {
            _this.Position = (props.Position) ? new IfcAxis2Placement3D(props.Position) : undefined;
            _this.PolygonalBoundary = (props.PolygonalBoundary) ? new IfcBoundedCurve(props.PolygonalBoundary) : undefined;
        }
        return _this;
    }
    return IfcPolygonalBoundedHalfSpace;
}(IfcHalfSpaceSolid));
exports.IfcPolygonalBoundedHalfSpace = IfcPolygonalBoundedHalfSpace;
var IfcPolygonalFaceSet = /** @class */ (function (_super) {
    __extends(IfcPolygonalFaceSet, _super);
    function IfcPolygonalFaceSet(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPolygonalFaceSet";
        if (props) {
            _this.Faces = (_a = props.Faces) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcIndexedPolygonalFace(o); });
        }
        return _this;
    }
    return IfcPolygonalFaceSet;
}(IfcTessellatedFaceSet));
exports.IfcPolygonalFaceSet = IfcPolygonalFaceSet;
var IfcPresentationLayerWithStyle = /** @class */ (function (_super) {
    __extends(IfcPresentationLayerWithStyle, _super);
    function IfcPresentationLayerWithStyle(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcPresentationLayerWithStyle";
        if (props) {
            _this.LayerStyles = (_a = props.LayerStyles) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPresentationStyle(o); });
        }
        return _this;
    }
    return IfcPresentationLayerWithStyle;
}(IfcPresentationLayerAssignment));
exports.IfcPresentationLayerWithStyle = IfcPresentationLayerWithStyle;
var IfcProductDefinitionShape = /** @class */ (function (_super) {
    __extends(IfcProductDefinitionShape, _super);
    function IfcProductDefinitionShape(props) {
        var _a;
        var _this = _super.call(this) || this;
        _this["@class"] = ".IfcProductDefinitionShape";
        if (props) {
            _this.HasShapeAspects = (_a = props.HasShapeAspects) === null || _a === void 0 ? void 0 : _a.map(function (o) { return o; });
        }
        return _this;
    }
    return IfcProductDefinitionShape;
}(IfcProductRepresentation));
exports.IfcProductDefinitionShape = IfcProductDefinitionShape;
var IfcRegularTimeSeries = /** @class */ (function (_super) {
    __extends(IfcRegularTimeSeries, _super);
    function IfcRegularTimeSeries(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRegularTimeSeries";
        if (props) {
            _this.Values = (_a = props.Values) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcIrregularTimeSeriesValue(o); });
        }
        return _this;
    }
    return IfcRegularTimeSeries;
}(IfcTimeSeries));
exports.IfcRegularTimeSeries = IfcRegularTimeSeries;
var IfcRelAssignsToActor = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToActor, _super);
    function IfcRelAssignsToActor(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToActor";
        if (props) {
            _this.RelatingActor = (props.RelatingActor) ? new IfcActor(props.RelatingActor) : undefined;
            _this.ActingRole = (props.ActingRole) ? new IfcActorRole(props.ActingRole) : undefined;
        }
        return _this;
    }
    return IfcRelAssignsToActor;
}(IfcRelAssigns));
exports.IfcRelAssignsToActor = IfcRelAssignsToActor;
var IfcRelAssignsToControl = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToControl, _super);
    function IfcRelAssignsToControl(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToControl";
        if (props) {
            _this.RelatingControl = (props.RelatingControl) ? new IfcControl(props.RelatingControl) : undefined;
        }
        return _this;
    }
    return IfcRelAssignsToControl;
}(IfcRelAssigns));
exports.IfcRelAssignsToControl = IfcRelAssignsToControl;
var IfcRelAssignsToGroup = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToGroup, _super);
    function IfcRelAssignsToGroup(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToGroup";
        if (props) {
            _this.RelatingGroup = (props.RelatingGroup) ? new IfcGroup(props.RelatingGroup) : undefined;
        }
        return _this;
    }
    return IfcRelAssignsToGroup;
}(IfcRelAssigns));
exports.IfcRelAssignsToGroup = IfcRelAssignsToGroup;
var IfcRelAssignsToProcess = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToProcess, _super);
    function IfcRelAssignsToProcess(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToProcess";
        if (props) {
            _this.QuantityInProcess = (props.QuantityInProcess) ? new IfcMeasureWithUnit(props.QuantityInProcess) : undefined;
        }
        return _this;
    }
    return IfcRelAssignsToProcess;
}(IfcRelAssigns));
exports.IfcRelAssignsToProcess = IfcRelAssignsToProcess;
var IfcRelAssignsToProduct = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToProduct, _super);
    function IfcRelAssignsToProduct(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToProduct";
        return _this;
    }
    return IfcRelAssignsToProduct;
}(IfcRelAssigns));
exports.IfcRelAssignsToProduct = IfcRelAssignsToProduct;
var IfcRelAssignsToResource = /** @class */ (function (_super) {
    __extends(IfcRelAssignsToResource, _super);
    function IfcRelAssignsToResource(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssignsToResource";
        return _this;
    }
    return IfcRelAssignsToResource;
}(IfcRelAssigns));
exports.IfcRelAssignsToResource = IfcRelAssignsToResource;
var IfcRelAssociatesApproval = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesApproval, _super);
    function IfcRelAssociatesApproval(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesApproval";
        if (props) {
            _this.RelatingApproval = (props.RelatingApproval) ? new IfcApproval(props.RelatingApproval) : undefined;
        }
        return _this;
    }
    return IfcRelAssociatesApproval;
}(IfcRelAssociates));
exports.IfcRelAssociatesApproval = IfcRelAssociatesApproval;
var IfcRelAssociatesClassification = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesClassification, _super);
    function IfcRelAssociatesClassification(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesClassification";
        return _this;
    }
    return IfcRelAssociatesClassification;
}(IfcRelAssociates));
exports.IfcRelAssociatesClassification = IfcRelAssociatesClassification;
var IfcRelAssociatesConstraint = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesConstraint, _super);
    function IfcRelAssociatesConstraint(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesConstraint";
        if (props) {
            _this.RelatingConstraint = (props.RelatingConstraint) ? new IfcConstraint(props.RelatingConstraint) : undefined;
        }
        return _this;
    }
    return IfcRelAssociatesConstraint;
}(IfcRelAssociates));
exports.IfcRelAssociatesConstraint = IfcRelAssociatesConstraint;
var IfcRelAssociatesDocument = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesDocument, _super);
    function IfcRelAssociatesDocument(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesDocument";
        return _this;
    }
    return IfcRelAssociatesDocument;
}(IfcRelAssociates));
exports.IfcRelAssociatesDocument = IfcRelAssociatesDocument;
var IfcRelAssociatesLibrary = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesLibrary, _super);
    function IfcRelAssociatesLibrary(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesLibrary";
        return _this;
    }
    return IfcRelAssociatesLibrary;
}(IfcRelAssociates));
exports.IfcRelAssociatesLibrary = IfcRelAssociatesLibrary;
var IfcRelAssociatesMaterial = /** @class */ (function (_super) {
    __extends(IfcRelAssociatesMaterial, _super);
    function IfcRelAssociatesMaterial(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelAssociatesMaterial";
        return _this;
    }
    return IfcRelAssociatesMaterial;
}(IfcRelAssociates));
exports.IfcRelAssociatesMaterial = IfcRelAssociatesMaterial;
var IfcRelConnectsWithEccentricity = /** @class */ (function (_super) {
    __extends(IfcRelConnectsWithEccentricity, _super);
    function IfcRelConnectsWithEccentricity(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsWithEccentricity";
        if (props) {
            _this.ConnectionConstraint = (props.ConnectionConstraint) ? new IfcConnectionGeometry(props.ConnectionConstraint) : undefined;
        }
        return _this;
    }
    return IfcRelConnectsWithEccentricity;
}(IfcRelConnectsStructuralMember));
exports.IfcRelConnectsWithEccentricity = IfcRelConnectsWithEccentricity;
var IfcRelConnectsWithRealizingElements = /** @class */ (function (_super) {
    __extends(IfcRelConnectsWithRealizingElements, _super);
    function IfcRelConnectsWithRealizingElements(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelConnectsWithRealizingElements";
        if (props) {
            _this.RealizingElements = (_a = props.RealizingElements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcElement(o); });
        }
        return _this;
    }
    return IfcRelConnectsWithRealizingElements;
}(IfcRelConnectsElements));
exports.IfcRelConnectsWithRealizingElements = IfcRelConnectsWithRealizingElements;
var IfcRelSpaceBoundary1stLevel = /** @class */ (function (_super) {
    __extends(IfcRelSpaceBoundary1stLevel, _super);
    function IfcRelSpaceBoundary1stLevel(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelSpaceBoundary1stLevel";
        if (props) {
            _this.ParentBoundary = (props.ParentBoundary) ? new IfcRelSpaceBoundary1stLevel(props.ParentBoundary) : undefined;
        }
        return _this;
    }
    return IfcRelSpaceBoundary1stLevel;
}(IfcRelSpaceBoundary));
exports.IfcRelSpaceBoundary1stLevel = IfcRelSpaceBoundary1stLevel;
var IfcRevolvedAreaSolid = /** @class */ (function (_super) {
    __extends(IfcRevolvedAreaSolid, _super);
    function IfcRevolvedAreaSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRevolvedAreaSolid";
        if (props) {
            _this.Axis = (props.Axis) ? new IfcAxis1Placement(props.Axis) : undefined;
        }
        return _this;
    }
    return IfcRevolvedAreaSolid;
}(IfcSweptAreaSolid));
exports.IfcRevolvedAreaSolid = IfcRevolvedAreaSolid;
var IfcStructuralCurveConnection = /** @class */ (function (_super) {
    __extends(IfcStructuralCurveConnection, _super);
    function IfcStructuralCurveConnection(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralCurveConnection";
        if (props) {
            _this.Axis = (props.Axis) ? new IfcDirection(props.Axis) : undefined;
        }
        return _this;
    }
    return IfcStructuralCurveConnection;
}(IfcStructuralConnection));
exports.IfcStructuralCurveConnection = IfcStructuralCurveConnection;
var IfcStructuralPointConnection = /** @class */ (function (_super) {
    __extends(IfcStructuralPointConnection, _super);
    function IfcStructuralPointConnection(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralPointConnection";
        if (props) {
            _this.ConditionCoordinateSystem = (props.ConditionCoordinateSystem) ? new IfcAxis2Placement3D(props.ConditionCoordinateSystem) : undefined;
        }
        return _this;
    }
    return IfcStructuralPointConnection;
}(IfcStructuralConnection));
exports.IfcStructuralPointConnection = IfcStructuralPointConnection;
var IfcStyledItem = /** @class */ (function (_super) {
    __extends(IfcStyledItem, _super);
    function IfcStyledItem(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStyledItem";
        return _this;
    }
    return IfcStyledItem;
}(IfcRepresentationItem));
exports.IfcStyledItem = IfcStyledItem;
var IfcSubedge = /** @class */ (function (_super) {
    __extends(IfcSubedge, _super);
    function IfcSubedge(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSubedge";
        if (props) {
            _this.ParentEdge = (props.ParentEdge) ? new IfcEdge(props.ParentEdge) : undefined;
        }
        return _this;
    }
    return IfcSubedge;
}(IfcEdge));
exports.IfcSubedge = IfcSubedge;
var IfcSurfaceCurveSweptAreaSolid = /** @class */ (function (_super) {
    __extends(IfcSurfaceCurveSweptAreaSolid, _super);
    function IfcSurfaceCurveSweptAreaSolid(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceCurveSweptAreaSolid";
        if (props) {
            _this.Directrix = (props.Directrix) ? new IfcCurve(props.Directrix) : undefined;
            _this.ReferenceSurface = (props.ReferenceSurface) ? new IfcSurface(props.ReferenceSurface) : undefined;
        }
        return _this;
    }
    return IfcSurfaceCurveSweptAreaSolid;
}(IfcSweptAreaSolid));
exports.IfcSurfaceCurveSweptAreaSolid = IfcSurfaceCurveSweptAreaSolid;
var IfcSurfaceOfLinearExtrusion = /** @class */ (function (_super) {
    __extends(IfcSurfaceOfLinearExtrusion, _super);
    function IfcSurfaceOfLinearExtrusion(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceOfLinearExtrusion";
        if (props) {
            _this.ExtrudedDirection = (props.ExtrudedDirection) ? new IfcDirection(props.ExtrudedDirection) : undefined;
        }
        return _this;
    }
    return IfcSurfaceOfLinearExtrusion;
}(IfcSweptSurface));
exports.IfcSurfaceOfLinearExtrusion = IfcSurfaceOfLinearExtrusion;
var IfcSurfaceOfRevolution = /** @class */ (function (_super) {
    __extends(IfcSurfaceOfRevolution, _super);
    function IfcSurfaceOfRevolution(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceOfRevolution";
        if (props) {
            _this.AxisPosition = (props.AxisPosition) ? new IfcAxis1Placement(props.AxisPosition) : undefined;
        }
        return _this;
    }
    return IfcSurfaceOfRevolution;
}(IfcSweptSurface));
exports.IfcSurfaceOfRevolution = IfcSurfaceOfRevolution;
var IfcSurfaceStyleRendering = /** @class */ (function (_super) {
    __extends(IfcSurfaceStyleRendering, _super);
    function IfcSurfaceStyleRendering(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSurfaceStyleRendering";
        return _this;
    }
    return IfcSurfaceStyleRendering;
}(IfcSurfaceStyleShading));
exports.IfcSurfaceStyleRendering = IfcSurfaceStyleRendering;
var IfcTextLiteralWithExtent = /** @class */ (function (_super) {
    __extends(IfcTextLiteralWithExtent, _super);
    function IfcTextLiteralWithExtent(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextLiteralWithExtent";
        if (props) {
            _this.Extent = (props.Extent) ? new IfcPlanarExtent(props.Extent) : undefined;
        }
        return _this;
    }
    return IfcTextLiteralWithExtent;
}(IfcTextLiteral));
exports.IfcTextLiteralWithExtent = IfcTextLiteralWithExtent;
var IfcTextureMap = /** @class */ (function (_super) {
    __extends(IfcTextureMap, _super);
    function IfcTextureMap(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTextureMap";
        if (props) {
            _this.Vertices = (_a = props.Vertices) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcTextureVertex(o); });
            _this.MappedTo = (_b = props.MappedTo) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcFaceBound(o); });
        }
        return _this;
    }
    return IfcTextureMap;
}(IfcTextureCoordinate));
exports.IfcTextureMap = IfcTextureMap;
var IfcArbitraryProfileDefWithVoids = /** @class */ (function (_super) {
    __extends(IfcArbitraryProfileDefWithVoids, _super);
    function IfcArbitraryProfileDefWithVoids(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcArbitraryProfileDefWithVoids";
        if (props) {
            _this.InnerCurves = (_a = props.InnerCurves) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcCurve(o); });
        }
        return _this;
    }
    return IfcArbitraryProfileDefWithVoids;
}(IfcArbitraryClosedProfileDef));
exports.IfcArbitraryProfileDefWithVoids = IfcArbitraryProfileDefWithVoids;
var IfcContext = /** @class */ (function (_super) {
    __extends(IfcContext, _super);
    function IfcContext(props) {
        var _a, _b, _c;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcContext";
        if (props) {
            _this.RepresentationContexts = (_a = props.RepresentationContexts) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRepresentationContext(o); });
            _this.UnitsInContext = (props.UnitsInContext) ? new IfcUnitAssignment(props.UnitsInContext) : undefined;
            _this.IsDefinedBy = (_b = props.IsDefinedBy) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcRelDefinesByProperties(o); });
            _this.Declares = (_c = props.Declares) === null || _c === void 0 ? void 0 : _c.map(function (o) { return new IfcRelDeclares(o); });
        }
        return _this;
    }
    return IfcContext;
}(IfcObjectDefinition));
exports.IfcContext = IfcContext;
var IfcExtrudedAreaSolidTapered = /** @class */ (function (_super) {
    __extends(IfcExtrudedAreaSolidTapered, _super);
    function IfcExtrudedAreaSolidTapered(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcExtrudedAreaSolidTapered";
        if (props) {
            _this.EndSweptArea = (_a = props.EndSweptArea) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcExtrudedAreaSolidTapered;
}(IfcExtrudedAreaSolid));
exports.IfcExtrudedAreaSolidTapered = IfcExtrudedAreaSolidTapered;
var IfcLightSourceSpot = /** @class */ (function (_super) {
    __extends(IfcLightSourceSpot, _super);
    function IfcLightSourceSpot(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcLightSourceSpot";
        if (props) {
            _this.Orientation = (props.Orientation) ? new IfcDirection(props.Orientation) : undefined;
        }
        return _this;
    }
    return IfcLightSourceSpot;
}(IfcLightSourcePositional));
exports.IfcLightSourceSpot = IfcLightSourceSpot;
var IfcObject = /** @class */ (function (_super) {
    __extends(IfcObject, _super);
    function IfcObject(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcObject";
        if (props) {
            _this.IsDeclaredBy = (_a = props.IsDeclaredBy) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcObjectDefinition(o); });
            _this.IsTypedBy = (props.IsTypedBy) ? new IfcRelDefinesByType(props.IsTypedBy) : undefined;
            _this.IsDefinedBy = (_b = props.IsDefinedBy) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcRelDefinesByProperties(o); });
        }
        return _this;
    }
    return IfcObject;
}(IfcObjectDefinition));
exports.IfcObject = IfcObject;
var IfcRelSpaceBoundary2ndLevel = /** @class */ (function (_super) {
    __extends(IfcRelSpaceBoundary2ndLevel, _super);
    function IfcRelSpaceBoundary2ndLevel(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRelSpaceBoundary2ndLevel";
        if (props) {
            _this.CorrespondingBoundary = (props.CorrespondingBoundary) ? new IfcRelSpaceBoundary2ndLevel(props.CorrespondingBoundary) : undefined;
        }
        return _this;
    }
    return IfcRelSpaceBoundary2ndLevel;
}(IfcRelSpaceBoundary1stLevel));
exports.IfcRelSpaceBoundary2ndLevel = IfcRelSpaceBoundary2ndLevel;
var IfcRevolvedAreaSolidTapered = /** @class */ (function (_super) {
    __extends(IfcRevolvedAreaSolidTapered, _super);
    function IfcRevolvedAreaSolidTapered(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcRevolvedAreaSolidTapered";
        if (props) {
            _this.EndSweptArea = (_a = props.EndSweptArea) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcProperty(o); });
        }
        return _this;
    }
    return IfcRevolvedAreaSolidTapered;
}(IfcRevolvedAreaSolid));
exports.IfcRevolvedAreaSolidTapered = IfcRevolvedAreaSolidTapered;
var IfcTypeObject = /** @class */ (function (_super) {
    __extends(IfcTypeObject, _super);
    function IfcTypeObject(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTypeObject";
        if (props) {
            _this.HasPropertySets = (_a = props.HasPropertySets) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcPropertySetDefinition(o); });
        }
        return _this;
    }
    return IfcTypeObject;
}(IfcObjectDefinition));
exports.IfcTypeObject = IfcTypeObject;
var IfcActor = /** @class */ (function (_super) {
    __extends(IfcActor, _super);
    function IfcActor(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcActor";
        return _this;
    }
    return IfcActor;
}(IfcObject));
exports.IfcActor = IfcActor;
var IfcProduct = /** @class */ (function (_super) {
    __extends(IfcProduct, _super);
    function IfcProduct(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcProduct";
        if (props) {
            _this.ObjectPlacement = (props.ObjectPlacement) ? new IfcObjectPlacement(props.ObjectPlacement) : undefined;
            _this.Representation = (_a = props.Representation) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRepresentation(o); });
        }
        return _this;
    }
    return IfcProduct;
}(IfcObject));
exports.IfcProduct = IfcProduct;
var IfcTypeProduct = /** @class */ (function (_super) {
    __extends(IfcTypeProduct, _super);
    function IfcTypeProduct(props) {
        var _a;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcTypeProduct";
        if (props) {
            _this.RepresentationMaps = (_a = props.RepresentationMaps) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRepresentationMap(o); });
        }
        return _this;
    }
    return IfcTypeProduct;
}(IfcTypeObject));
exports.IfcTypeProduct = IfcTypeProduct;
var IfcElement = /** @class */ (function (_super) {
    __extends(IfcElement, _super);
    function IfcElement(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcElement";
        if (props) {
            _this.HasProjections = (props.HasProjections) ? new IfcRelProjectsElement(props.HasProjections) : undefined;
            _this.HasOpenings = (props.HasOpenings) ? new IfcRelVoidsElement(props.HasOpenings) : undefined;
        }
        return _this;
    }
    return IfcElement;
}(IfcProduct));
exports.IfcElement = IfcElement;
var IfcGrid = /** @class */ (function (_super) {
    __extends(IfcGrid, _super);
    function IfcGrid(props) {
        var _a, _b, _c;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcGrid";
        if (props) {
            _this.UAxes = (_a = props.UAxes) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcGridAxis(o); });
            _this.VAxes = (_b = props.VAxes) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcGridAxis(o); });
            _this.WAxes = (_c = props.WAxes) === null || _c === void 0 ? void 0 : _c.map(function (o) { return new IfcGridAxis(o); });
        }
        return _this;
    }
    return IfcGrid;
}(IfcProduct));
exports.IfcGrid = IfcGrid;
var IfcSpatialElement = /** @class */ (function (_super) {
    __extends(IfcSpatialElement, _super);
    function IfcSpatialElement(props) {
        var _a, _b;
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcSpatialElement";
        if (props) {
            _this.ContainsElements = (_a = props.ContainsElements) === null || _a === void 0 ? void 0 : _a.map(function (o) { return new IfcRelContainedInSpatialStructure(o); });
            _this.ReferencesElements = (_b = props.ReferencesElements) === null || _b === void 0 ? void 0 : _b.map(function (o) { return new IfcRelReferencedInSpatialStructure(o); });
        }
        return _this;
    }
    return IfcSpatialElement;
}(IfcProduct));
exports.IfcSpatialElement = IfcSpatialElement;
var IfcStructuralActivity = /** @class */ (function (_super) {
    __extends(IfcStructuralActivity, _super);
    function IfcStructuralActivity(props) {
        var _this = _super.call(this, props) || this;
        _this["@class"] = ".IfcStructuralActivity";
        if (props) {
            _this.AppliedLoad = (props.AppliedLoad) ? new IfcStructuralLoad(props.AppliedLoad) : undefined;
        }
        return _this;
    }
    return IfcStructuralActivity;
}(IfcProduct));
exports.IfcStructuralActivity = IfcStructuralActivity;
var IfcActionRequestTypeEnum;
(function (IfcActionRequestTypeEnum) {
    IfcActionRequestTypeEnum["email"] = "email";
    IfcActionRequestTypeEnum["fax"] = "fax";
    IfcActionRequestTypeEnum["phone"] = "phone";
    IfcActionRequestTypeEnum["post"] = "post";
    IfcActionRequestTypeEnum["verbal"] = "verbal";
    IfcActionRequestTypeEnum["userdefined"] = "userdefined";
    IfcActionRequestTypeEnum["notdefined"] = "notdefined";
})(IfcActionRequestTypeEnum || (IfcActionRequestTypeEnum = {}));
var IfcActionSourceTypeEnum;
(function (IfcActionSourceTypeEnum) {
    IfcActionSourceTypeEnum["dead_load_g"] = "dead_load_g";
    IfcActionSourceTypeEnum["completion_g1"] = "completion_g1";
    IfcActionSourceTypeEnum["live_load_q"] = "live_load_q";
    IfcActionSourceTypeEnum["snow_s"] = "snow_s";
    IfcActionSourceTypeEnum["wind_w"] = "wind_w";
    IfcActionSourceTypeEnum["prestressing_p"] = "prestressing_p";
    IfcActionSourceTypeEnum["settlement_u"] = "settlement_u";
    IfcActionSourceTypeEnum["temperature_t"] = "temperature_t";
    IfcActionSourceTypeEnum["earthquake_e"] = "earthquake_e";
    IfcActionSourceTypeEnum["fire"] = "fire";
    IfcActionSourceTypeEnum["impulse"] = "impulse";
    IfcActionSourceTypeEnum["impact"] = "impact";
    IfcActionSourceTypeEnum["transport"] = "transport";
    IfcActionSourceTypeEnum["erection"] = "erection";
    IfcActionSourceTypeEnum["propping"] = "propping";
    IfcActionSourceTypeEnum["system_imperfection"] = "system_imperfection";
    IfcActionSourceTypeEnum["shrinkage"] = "shrinkage";
    IfcActionSourceTypeEnum["creep"] = "creep";
    IfcActionSourceTypeEnum["lack_of_fit"] = "lack_of_fit";
    IfcActionSourceTypeEnum["buoyancy"] = "buoyancy";
    IfcActionSourceTypeEnum["ice"] = "ice";
    IfcActionSourceTypeEnum["current"] = "current";
    IfcActionSourceTypeEnum["wave"] = "wave";
    IfcActionSourceTypeEnum["rain"] = "rain";
    IfcActionSourceTypeEnum["brakes"] = "brakes";
    IfcActionSourceTypeEnum["userdefined"] = "userdefined";
    IfcActionSourceTypeEnum["notdefined"] = "notdefined";
})(IfcActionSourceTypeEnum || (IfcActionSourceTypeEnum = {}));
var IfcActionTypeEnum;
(function (IfcActionTypeEnum) {
    IfcActionTypeEnum["permanent_g"] = "permanent_g";
    IfcActionTypeEnum["variable_q"] = "variable_q";
    IfcActionTypeEnum["extraordinary_a"] = "extraordinary_a";
    IfcActionTypeEnum["userdefined"] = "userdefined";
    IfcActionTypeEnum["notdefined"] = "notdefined";
})(IfcActionTypeEnum || (IfcActionTypeEnum = {}));
var IfcActuatorTypeEnum;
(function (IfcActuatorTypeEnum) {
    IfcActuatorTypeEnum["electricactuator"] = "electricactuator";
    IfcActuatorTypeEnum["handoperatedactuator"] = "handoperatedactuator";
    IfcActuatorTypeEnum["hydraulicactuator"] = "hydraulicactuator";
    IfcActuatorTypeEnum["pneumaticactuator"] = "pneumaticactuator";
    IfcActuatorTypeEnum["thermostaticactuator"] = "thermostaticactuator";
    IfcActuatorTypeEnum["userdefined"] = "userdefined";
    IfcActuatorTypeEnum["notdefined"] = "notdefined";
})(IfcActuatorTypeEnum || (IfcActuatorTypeEnum = {}));
var IfcAddressTypeEnum;
(function (IfcAddressTypeEnum) {
    IfcAddressTypeEnum["office"] = "office";
    IfcAddressTypeEnum["site"] = "site";
    IfcAddressTypeEnum["home"] = "home";
    IfcAddressTypeEnum["distributionpoint"] = "distributionpoint";
    IfcAddressTypeEnum["userdefined"] = "userdefined";
})(IfcAddressTypeEnum || (IfcAddressTypeEnum = {}));
var IfcAirTerminalBoxTypeEnum;
(function (IfcAirTerminalBoxTypeEnum) {
    IfcAirTerminalBoxTypeEnum["constantflow"] = "constantflow";
    IfcAirTerminalBoxTypeEnum["variableflowpressuredependant"] = "variableflowpressuredependant";
    IfcAirTerminalBoxTypeEnum["variableflowpressureindependant"] = "variableflowpressureindependant";
    IfcAirTerminalBoxTypeEnum["userdefined"] = "userdefined";
    IfcAirTerminalBoxTypeEnum["notdefined"] = "notdefined";
})(IfcAirTerminalBoxTypeEnum || (IfcAirTerminalBoxTypeEnum = {}));
var IfcAirTerminalTypeEnum;
(function (IfcAirTerminalTypeEnum) {
    IfcAirTerminalTypeEnum["diffuser"] = "diffuser";
    IfcAirTerminalTypeEnum["grille"] = "grille";
    IfcAirTerminalTypeEnum["louvre"] = "louvre";
    IfcAirTerminalTypeEnum["register"] = "register";
    IfcAirTerminalTypeEnum["userdefined"] = "userdefined";
    IfcAirTerminalTypeEnum["notdefined"] = "notdefined";
})(IfcAirTerminalTypeEnum || (IfcAirTerminalTypeEnum = {}));
var IfcAirToAirHeatRecoveryTypeEnum;
(function (IfcAirToAirHeatRecoveryTypeEnum) {
    IfcAirToAirHeatRecoveryTypeEnum["fixedplatecounterflowexchanger"] = "fixedplatecounterflowexchanger";
    IfcAirToAirHeatRecoveryTypeEnum["fixedplatecrossflowexchanger"] = "fixedplatecrossflowexchanger";
    IfcAirToAirHeatRecoveryTypeEnum["fixedplateparallelflowexchanger"] = "fixedplateparallelflowexchanger";
    IfcAirToAirHeatRecoveryTypeEnum["rotarywheel"] = "rotarywheel";
    IfcAirToAirHeatRecoveryTypeEnum["runaroundcoilloop"] = "runaroundcoilloop";
    IfcAirToAirHeatRecoveryTypeEnum["heatpipe"] = "heatpipe";
    IfcAirToAirHeatRecoveryTypeEnum["twintowerenthalpyrecoveryloops"] = "twintowerenthalpyrecoveryloops";
    IfcAirToAirHeatRecoveryTypeEnum["thermosiphonsealedtubeheatexchangers"] = "thermosiphonsealedtubeheatexchangers";
    IfcAirToAirHeatRecoveryTypeEnum["thermosiphoncoiltypeheatexchangers"] = "thermosiphoncoiltypeheatexchangers";
    IfcAirToAirHeatRecoveryTypeEnum["userdefined"] = "userdefined";
    IfcAirToAirHeatRecoveryTypeEnum["notdefined"] = "notdefined";
})(IfcAirToAirHeatRecoveryTypeEnum || (IfcAirToAirHeatRecoveryTypeEnum = {}));
var IfcAlarmTypeEnum;
(function (IfcAlarmTypeEnum) {
    IfcAlarmTypeEnum["bell"] = "bell";
    IfcAlarmTypeEnum["breakglassbutton"] = "breakglassbutton";
    IfcAlarmTypeEnum["light"] = "light";
    IfcAlarmTypeEnum["manualpullbox"] = "manualpullbox";
    IfcAlarmTypeEnum["siren"] = "siren";
    IfcAlarmTypeEnum["whistle"] = "whistle";
    IfcAlarmTypeEnum["userdefined"] = "userdefined";
    IfcAlarmTypeEnum["notdefined"] = "notdefined";
})(IfcAlarmTypeEnum || (IfcAlarmTypeEnum = {}));
var IfcAnalysisModelTypeEnum;
(function (IfcAnalysisModelTypeEnum) {
    IfcAnalysisModelTypeEnum["in_plane_loading_2d"] = "in_plane_loading_2d";
    IfcAnalysisModelTypeEnum["out_plane_loading_2d"] = "out_plane_loading_2d";
    IfcAnalysisModelTypeEnum["loading_3d"] = "loading_3d";
    IfcAnalysisModelTypeEnum["userdefined"] = "userdefined";
    IfcAnalysisModelTypeEnum["notdefined"] = "notdefined";
})(IfcAnalysisModelTypeEnum || (IfcAnalysisModelTypeEnum = {}));
var IfcAnalysisTheoryTypeEnum;
(function (IfcAnalysisTheoryTypeEnum) {
    IfcAnalysisTheoryTypeEnum["first_order_theory"] = "first_order_theory";
    IfcAnalysisTheoryTypeEnum["second_order_theory"] = "second_order_theory";
    IfcAnalysisTheoryTypeEnum["third_order_theory"] = "third_order_theory";
    IfcAnalysisTheoryTypeEnum["full_nonlinear_theory"] = "full_nonlinear_theory";
    IfcAnalysisTheoryTypeEnum["userdefined"] = "userdefined";
    IfcAnalysisTheoryTypeEnum["notdefined"] = "notdefined";
})(IfcAnalysisTheoryTypeEnum || (IfcAnalysisTheoryTypeEnum = {}));
var IfcArithmeticOperatorEnum;
(function (IfcArithmeticOperatorEnum) {
    IfcArithmeticOperatorEnum["add"] = "add";
    IfcArithmeticOperatorEnum["divide"] = "divide";
    IfcArithmeticOperatorEnum["multiply"] = "multiply";
    IfcArithmeticOperatorEnum["subtract"] = "subtract";
})(IfcArithmeticOperatorEnum || (IfcArithmeticOperatorEnum = {}));
var IfcAssemblyPlaceEnum;
(function (IfcAssemblyPlaceEnum) {
    IfcAssemblyPlaceEnum["site"] = "site";
    IfcAssemblyPlaceEnum["factory"] = "factory";
    IfcAssemblyPlaceEnum["notdefined"] = "notdefined";
})(IfcAssemblyPlaceEnum || (IfcAssemblyPlaceEnum = {}));
var IfcAudioVisualApplianceTypeEnum;
(function (IfcAudioVisualApplianceTypeEnum) {
    IfcAudioVisualApplianceTypeEnum["amplifier"] = "amplifier";
    IfcAudioVisualApplianceTypeEnum["camera"] = "camera";
    IfcAudioVisualApplianceTypeEnum["display"] = "display";
    IfcAudioVisualApplianceTypeEnum["microphone"] = "microphone";
    IfcAudioVisualApplianceTypeEnum["player"] = "player";
    IfcAudioVisualApplianceTypeEnum["projector"] = "projector";
    IfcAudioVisualApplianceTypeEnum["receiver"] = "receiver";
    IfcAudioVisualApplianceTypeEnum["speaker"] = "speaker";
    IfcAudioVisualApplianceTypeEnum["switcher"] = "switcher";
    IfcAudioVisualApplianceTypeEnum["telephone"] = "telephone";
    IfcAudioVisualApplianceTypeEnum["tuner"] = "tuner";
    IfcAudioVisualApplianceTypeEnum["userdefined"] = "userdefined";
    IfcAudioVisualApplianceTypeEnum["notdefined"] = "notdefined";
})(IfcAudioVisualApplianceTypeEnum || (IfcAudioVisualApplianceTypeEnum = {}));
var IfcBSplineCurveForm;
(function (IfcBSplineCurveForm) {
    IfcBSplineCurveForm["polyline_form"] = "polyline_form";
    IfcBSplineCurveForm["circular_arc"] = "circular_arc";
    IfcBSplineCurveForm["elliptic_arc"] = "elliptic_arc";
    IfcBSplineCurveForm["parabolic_arc"] = "parabolic_arc";
    IfcBSplineCurveForm["hyperbolic_arc"] = "hyperbolic_arc";
    IfcBSplineCurveForm["unspecified"] = "unspecified";
})(IfcBSplineCurveForm || (IfcBSplineCurveForm = {}));
var IfcBSplineSurfaceForm;
(function (IfcBSplineSurfaceForm) {
    IfcBSplineSurfaceForm["plane_surf"] = "plane_surf";
    IfcBSplineSurfaceForm["cylindrical_surf"] = "cylindrical_surf";
    IfcBSplineSurfaceForm["conical_surf"] = "conical_surf";
    IfcBSplineSurfaceForm["spherical_surf"] = "spherical_surf";
    IfcBSplineSurfaceForm["toroidal_surf"] = "toroidal_surf";
    IfcBSplineSurfaceForm["surf_of_revolution"] = "surf_of_revolution";
    IfcBSplineSurfaceForm["ruled_surf"] = "ruled_surf";
    IfcBSplineSurfaceForm["generalised_cone"] = "generalised_cone";
    IfcBSplineSurfaceForm["quadric_surf"] = "quadric_surf";
    IfcBSplineSurfaceForm["surf_of_linear_extrusion"] = "surf_of_linear_extrusion";
    IfcBSplineSurfaceForm["unspecified"] = "unspecified";
})(IfcBSplineSurfaceForm || (IfcBSplineSurfaceForm = {}));
var IfcBeamTypeEnum;
(function (IfcBeamTypeEnum) {
    IfcBeamTypeEnum["beam"] = "beam";
    IfcBeamTypeEnum["joist"] = "joist";
    IfcBeamTypeEnum["hollowcore"] = "hollowcore";
    IfcBeamTypeEnum["lintel"] = "lintel";
    IfcBeamTypeEnum["spandrel"] = "spandrel";
    IfcBeamTypeEnum["t_beam"] = "t_beam";
    IfcBeamTypeEnum["userdefined"] = "userdefined";
    IfcBeamTypeEnum["notdefined"] = "notdefined";
})(IfcBeamTypeEnum || (IfcBeamTypeEnum = {}));
var IfcBenchmarkEnum;
(function (IfcBenchmarkEnum) {
    IfcBenchmarkEnum["greaterthan"] = "greaterthan";
    IfcBenchmarkEnum["greaterthanorequalto"] = "greaterthanorequalto";
    IfcBenchmarkEnum["lessthan"] = "lessthan";
    IfcBenchmarkEnum["lessthanorequalto"] = "lessthanorequalto";
    IfcBenchmarkEnum["equalto"] = "equalto";
    IfcBenchmarkEnum["notequalto"] = "notequalto";
    IfcBenchmarkEnum["includes"] = "includes";
    IfcBenchmarkEnum["notincludes"] = "notincludes";
    IfcBenchmarkEnum["includedin"] = "includedin";
    IfcBenchmarkEnum["notincludedin"] = "notincludedin";
})(IfcBenchmarkEnum || (IfcBenchmarkEnum = {}));
var IfcBoilerTypeEnum;
(function (IfcBoilerTypeEnum) {
    IfcBoilerTypeEnum["water"] = "water";
    IfcBoilerTypeEnum["steam"] = "steam";
    IfcBoilerTypeEnum["userdefined"] = "userdefined";
    IfcBoilerTypeEnum["notdefined"] = "notdefined";
})(IfcBoilerTypeEnum || (IfcBoilerTypeEnum = {}));
var IfcBooleanOperator;
(function (IfcBooleanOperator) {
    IfcBooleanOperator["union"] = "union";
    IfcBooleanOperator["intersection"] = "intersection";
    IfcBooleanOperator["difference"] = "difference";
})(IfcBooleanOperator || (IfcBooleanOperator = {}));
var IfcBuildingElementPartTypeEnum;
(function (IfcBuildingElementPartTypeEnum) {
    IfcBuildingElementPartTypeEnum["insulation"] = "insulation";
    IfcBuildingElementPartTypeEnum["precastpanel"] = "precastpanel";
    IfcBuildingElementPartTypeEnum["userdefined"] = "userdefined";
    IfcBuildingElementPartTypeEnum["notdefined"] = "notdefined";
})(IfcBuildingElementPartTypeEnum || (IfcBuildingElementPartTypeEnum = {}));
var IfcBuildingElementProxyTypeEnum;
(function (IfcBuildingElementProxyTypeEnum) {
    IfcBuildingElementProxyTypeEnum["complex"] = "complex";
    IfcBuildingElementProxyTypeEnum["element"] = "element";
    IfcBuildingElementProxyTypeEnum["partial"] = "partial";
    IfcBuildingElementProxyTypeEnum["provisionforvoid"] = "provisionforvoid";
    IfcBuildingElementProxyTypeEnum["provisionforspace"] = "provisionforspace";
    IfcBuildingElementProxyTypeEnum["userdefined"] = "userdefined";
    IfcBuildingElementProxyTypeEnum["notdefined"] = "notdefined";
})(IfcBuildingElementProxyTypeEnum || (IfcBuildingElementProxyTypeEnum = {}));
var IfcBuildingSystemTypeEnum;
(function (IfcBuildingSystemTypeEnum) {
    IfcBuildingSystemTypeEnum["fenestration"] = "fenestration";
    IfcBuildingSystemTypeEnum["foundation"] = "foundation";
    IfcBuildingSystemTypeEnum["loadbearing"] = "loadbearing";
    IfcBuildingSystemTypeEnum["outershell"] = "outershell";
    IfcBuildingSystemTypeEnum["shading"] = "shading";
    IfcBuildingSystemTypeEnum["transport"] = "transport";
    IfcBuildingSystemTypeEnum["userdefined"] = "userdefined";
    IfcBuildingSystemTypeEnum["notdefined"] = "notdefined";
})(IfcBuildingSystemTypeEnum || (IfcBuildingSystemTypeEnum = {}));
var IfcBurnerTypeEnum;
(function (IfcBurnerTypeEnum) {
    IfcBurnerTypeEnum["userdefined"] = "userdefined";
    IfcBurnerTypeEnum["notdefined"] = "notdefined";
})(IfcBurnerTypeEnum || (IfcBurnerTypeEnum = {}));
var IfcCableCarrierFittingTypeEnum;
(function (IfcCableCarrierFittingTypeEnum) {
    IfcCableCarrierFittingTypeEnum["bend"] = "bend";
    IfcCableCarrierFittingTypeEnum["cross"] = "cross";
    IfcCableCarrierFittingTypeEnum["reducer"] = "reducer";
    IfcCableCarrierFittingTypeEnum["tee"] = "tee";
    IfcCableCarrierFittingTypeEnum["userdefined"] = "userdefined";
    IfcCableCarrierFittingTypeEnum["notdefined"] = "notdefined";
})(IfcCableCarrierFittingTypeEnum || (IfcCableCarrierFittingTypeEnum = {}));
var IfcCableCarrierSegmentTypeEnum;
(function (IfcCableCarrierSegmentTypeEnum) {
    IfcCableCarrierSegmentTypeEnum["cableladdersegment"] = "cableladdersegment";
    IfcCableCarrierSegmentTypeEnum["cabletraysegment"] = "cabletraysegment";
    IfcCableCarrierSegmentTypeEnum["cabletrunkingsegment"] = "cabletrunkingsegment";
    IfcCableCarrierSegmentTypeEnum["conduitsegment"] = "conduitsegment";
    IfcCableCarrierSegmentTypeEnum["userdefined"] = "userdefined";
    IfcCableCarrierSegmentTypeEnum["notdefined"] = "notdefined";
})(IfcCableCarrierSegmentTypeEnum || (IfcCableCarrierSegmentTypeEnum = {}));
var IfcCableFittingTypeEnum;
(function (IfcCableFittingTypeEnum) {
    IfcCableFittingTypeEnum["connector"] = "connector";
    IfcCableFittingTypeEnum["entry"] = "entry";
    IfcCableFittingTypeEnum["exit"] = "exit";
    IfcCableFittingTypeEnum["junction"] = "junction";
    IfcCableFittingTypeEnum["transition"] = "transition";
    IfcCableFittingTypeEnum["userdefined"] = "userdefined";
    IfcCableFittingTypeEnum["notdefined"] = "notdefined";
})(IfcCableFittingTypeEnum || (IfcCableFittingTypeEnum = {}));
var IfcCableSegmentTypeEnum;
(function (IfcCableSegmentTypeEnum) {
    IfcCableSegmentTypeEnum["busbarsegment"] = "busbarsegment";
    IfcCableSegmentTypeEnum["cablesegment"] = "cablesegment";
    IfcCableSegmentTypeEnum["conductorsegment"] = "conductorsegment";
    IfcCableSegmentTypeEnum["coresegment"] = "coresegment";
    IfcCableSegmentTypeEnum["userdefined"] = "userdefined";
    IfcCableSegmentTypeEnum["notdefined"] = "notdefined";
})(IfcCableSegmentTypeEnum || (IfcCableSegmentTypeEnum = {}));
var IfcChangeActionEnum;
(function (IfcChangeActionEnum) {
    IfcChangeActionEnum["nochange"] = "nochange";
    IfcChangeActionEnum["modified"] = "modified";
    IfcChangeActionEnum["added"] = "added";
    IfcChangeActionEnum["deleted"] = "deleted";
    IfcChangeActionEnum["notdefined"] = "notdefined";
})(IfcChangeActionEnum || (IfcChangeActionEnum = {}));
var IfcChillerTypeEnum;
(function (IfcChillerTypeEnum) {
    IfcChillerTypeEnum["aircooled"] = "aircooled";
    IfcChillerTypeEnum["watercooled"] = "watercooled";
    IfcChillerTypeEnum["heatrecovery"] = "heatrecovery";
    IfcChillerTypeEnum["userdefined"] = "userdefined";
    IfcChillerTypeEnum["notdefined"] = "notdefined";
})(IfcChillerTypeEnum || (IfcChillerTypeEnum = {}));
var IfcChimneyTypeEnum;
(function (IfcChimneyTypeEnum) {
    IfcChimneyTypeEnum["userdefined"] = "userdefined";
    IfcChimneyTypeEnum["notdefined"] = "notdefined";
})(IfcChimneyTypeEnum || (IfcChimneyTypeEnum = {}));
var IfcCoilTypeEnum;
(function (IfcCoilTypeEnum) {
    IfcCoilTypeEnum["dxcoolingcoil"] = "dxcoolingcoil";
    IfcCoilTypeEnum["electricheatingcoil"] = "electricheatingcoil";
    IfcCoilTypeEnum["gasheatingcoil"] = "gasheatingcoil";
    IfcCoilTypeEnum["hydroniccoil"] = "hydroniccoil";
    IfcCoilTypeEnum["steamheatingcoil"] = "steamheatingcoil";
    IfcCoilTypeEnum["watercoolingcoil"] = "watercoolingcoil";
    IfcCoilTypeEnum["waterheatingcoil"] = "waterheatingcoil";
    IfcCoilTypeEnum["userdefined"] = "userdefined";
    IfcCoilTypeEnum["notdefined"] = "notdefined";
})(IfcCoilTypeEnum || (IfcCoilTypeEnum = {}));
var IfcColumnTypeEnum;
(function (IfcColumnTypeEnum) {
    IfcColumnTypeEnum["column"] = "column";
    IfcColumnTypeEnum["pilaster"] = "pilaster";
    IfcColumnTypeEnum["userdefined"] = "userdefined";
    IfcColumnTypeEnum["notdefined"] = "notdefined";
})(IfcColumnTypeEnum || (IfcColumnTypeEnum = {}));
var IfcCommunicationsApplianceTypeEnum;
(function (IfcCommunicationsApplianceTypeEnum) {
    IfcCommunicationsApplianceTypeEnum["antenna"] = "antenna";
    IfcCommunicationsApplianceTypeEnum["computer"] = "computer";
    IfcCommunicationsApplianceTypeEnum["fax"] = "fax";
    IfcCommunicationsApplianceTypeEnum["gateway"] = "gateway";
    IfcCommunicationsApplianceTypeEnum["modem"] = "modem";
    IfcCommunicationsApplianceTypeEnum["networkappliance"] = "networkappliance";
    IfcCommunicationsApplianceTypeEnum["networkbridge"] = "networkbridge";
    IfcCommunicationsApplianceTypeEnum["networkhub"] = "networkhub";
    IfcCommunicationsApplianceTypeEnum["printer"] = "printer";
    IfcCommunicationsApplianceTypeEnum["repeater"] = "repeater";
    IfcCommunicationsApplianceTypeEnum["router"] = "router";
    IfcCommunicationsApplianceTypeEnum["scanner"] = "scanner";
    IfcCommunicationsApplianceTypeEnum["userdefined"] = "userdefined";
    IfcCommunicationsApplianceTypeEnum["notdefined"] = "notdefined";
})(IfcCommunicationsApplianceTypeEnum || (IfcCommunicationsApplianceTypeEnum = {}));
var IfcComplexPropertyTemplateTypeEnum;
(function (IfcComplexPropertyTemplateTypeEnum) {
    IfcComplexPropertyTemplateTypeEnum["p_complex"] = "p_complex";
    IfcComplexPropertyTemplateTypeEnum["q_complex"] = "q_complex";
})(IfcComplexPropertyTemplateTypeEnum || (IfcComplexPropertyTemplateTypeEnum = {}));
var IfcCompressorTypeEnum;
(function (IfcCompressorTypeEnum) {
    IfcCompressorTypeEnum["dynamic"] = "dynamic";
    IfcCompressorTypeEnum["reciprocating"] = "reciprocating";
    IfcCompressorTypeEnum["rotary"] = "rotary";
    IfcCompressorTypeEnum["scroll"] = "scroll";
    IfcCompressorTypeEnum["trochoidal"] = "trochoidal";
    IfcCompressorTypeEnum["singlestage"] = "singlestage";
    IfcCompressorTypeEnum["booster"] = "booster";
    IfcCompressorTypeEnum["opentype"] = "opentype";
    IfcCompressorTypeEnum["hermetic"] = "hermetic";
    IfcCompressorTypeEnum["semihermetic"] = "semihermetic";
    IfcCompressorTypeEnum["weldedshellhermetic"] = "weldedshellhermetic";
    IfcCompressorTypeEnum["rollingpiston"] = "rollingpiston";
    IfcCompressorTypeEnum["rotaryvane"] = "rotaryvane";
    IfcCompressorTypeEnum["singlescrew"] = "singlescrew";
    IfcCompressorTypeEnum["twinscrew"] = "twinscrew";
    IfcCompressorTypeEnum["userdefined"] = "userdefined";
    IfcCompressorTypeEnum["notdefined"] = "notdefined";
})(IfcCompressorTypeEnum || (IfcCompressorTypeEnum = {}));
var IfcCondenserTypeEnum;
(function (IfcCondenserTypeEnum) {
    IfcCondenserTypeEnum["aircooled"] = "aircooled";
    IfcCondenserTypeEnum["evaporativecooled"] = "evaporativecooled";
    IfcCondenserTypeEnum["watercooled"] = "watercooled";
    IfcCondenserTypeEnum["watercooledbrazedplate"] = "watercooledbrazedplate";
    IfcCondenserTypeEnum["watercooledshellcoil"] = "watercooledshellcoil";
    IfcCondenserTypeEnum["watercooledshelltube"] = "watercooledshelltube";
    IfcCondenserTypeEnum["watercooledtubeintube"] = "watercooledtubeintube";
    IfcCondenserTypeEnum["userdefined"] = "userdefined";
    IfcCondenserTypeEnum["notdefined"] = "notdefined";
})(IfcCondenserTypeEnum || (IfcCondenserTypeEnum = {}));
var IfcConnectionTypeEnum;
(function (IfcConnectionTypeEnum) {
    IfcConnectionTypeEnum["atpath"] = "atpath";
    IfcConnectionTypeEnum["atstart"] = "atstart";
    IfcConnectionTypeEnum["atend"] = "atend";
    IfcConnectionTypeEnum["notdefined"] = "notdefined";
})(IfcConnectionTypeEnum || (IfcConnectionTypeEnum = {}));
var IfcConstraintEnum;
(function (IfcConstraintEnum) {
    IfcConstraintEnum["hard"] = "hard";
    IfcConstraintEnum["soft"] = "soft";
    IfcConstraintEnum["advisory"] = "advisory";
    IfcConstraintEnum["userdefined"] = "userdefined";
    IfcConstraintEnum["notdefined"] = "notdefined";
})(IfcConstraintEnum || (IfcConstraintEnum = {}));
var IfcConstructionEquipmentResourceTypeEnum;
(function (IfcConstructionEquipmentResourceTypeEnum) {
    IfcConstructionEquipmentResourceTypeEnum["demolishing"] = "demolishing";
    IfcConstructionEquipmentResourceTypeEnum["earthmoving"] = "earthmoving";
    IfcConstructionEquipmentResourceTypeEnum["erecting"] = "erecting";
    IfcConstructionEquipmentResourceTypeEnum["heating"] = "heating";
    IfcConstructionEquipmentResourceTypeEnum["lighting"] = "lighting";
    IfcConstructionEquipmentResourceTypeEnum["paving"] = "paving";
    IfcConstructionEquipmentResourceTypeEnum["pumping"] = "pumping";
    IfcConstructionEquipmentResourceTypeEnum["transporting"] = "transporting";
    IfcConstructionEquipmentResourceTypeEnum["userdefined"] = "userdefined";
    IfcConstructionEquipmentResourceTypeEnum["notdefined"] = "notdefined";
})(IfcConstructionEquipmentResourceTypeEnum || (IfcConstructionEquipmentResourceTypeEnum = {}));
var IfcConstructionMaterialResourceTypeEnum;
(function (IfcConstructionMaterialResourceTypeEnum) {
    IfcConstructionMaterialResourceTypeEnum["aggregates"] = "aggregates";
    IfcConstructionMaterialResourceTypeEnum["concrete"] = "concrete";
    IfcConstructionMaterialResourceTypeEnum["drywall"] = "drywall";
    IfcConstructionMaterialResourceTypeEnum["fuel"] = "fuel";
    IfcConstructionMaterialResourceTypeEnum["gypsum"] = "gypsum";
    IfcConstructionMaterialResourceTypeEnum["masonry"] = "masonry";
    IfcConstructionMaterialResourceTypeEnum["metal"] = "metal";
    IfcConstructionMaterialResourceTypeEnum["plastic"] = "plastic";
    IfcConstructionMaterialResourceTypeEnum["wood"] = "wood";
    IfcConstructionMaterialResourceTypeEnum["notdefined"] = "notdefined";
    IfcConstructionMaterialResourceTypeEnum["userdefined"] = "userdefined";
})(IfcConstructionMaterialResourceTypeEnum || (IfcConstructionMaterialResourceTypeEnum = {}));
var IfcConstructionProductResourceTypeEnum;
(function (IfcConstructionProductResourceTypeEnum) {
    IfcConstructionProductResourceTypeEnum["assembly"] = "assembly";
    IfcConstructionProductResourceTypeEnum["formwork"] = "formwork";
    IfcConstructionProductResourceTypeEnum["userdefined"] = "userdefined";
    IfcConstructionProductResourceTypeEnum["notdefined"] = "notdefined";
})(IfcConstructionProductResourceTypeEnum || (IfcConstructionProductResourceTypeEnum = {}));
var IfcControllerTypeEnum;
(function (IfcControllerTypeEnum) {
    IfcControllerTypeEnum["floating"] = "floating";
    IfcControllerTypeEnum["programmable"] = "programmable";
    IfcControllerTypeEnum["proportional"] = "proportional";
    IfcControllerTypeEnum["multiposition"] = "multiposition";
    IfcControllerTypeEnum["twoposition"] = "twoposition";
    IfcControllerTypeEnum["userdefined"] = "userdefined";
    IfcControllerTypeEnum["notdefined"] = "notdefined";
})(IfcControllerTypeEnum || (IfcControllerTypeEnum = {}));
var IfcCooledBeamTypeEnum;
(function (IfcCooledBeamTypeEnum) {
    IfcCooledBeamTypeEnum["active"] = "active";
    IfcCooledBeamTypeEnum["passive"] = "passive";
    IfcCooledBeamTypeEnum["userdefined"] = "userdefined";
    IfcCooledBeamTypeEnum["notdefined"] = "notdefined";
})(IfcCooledBeamTypeEnum || (IfcCooledBeamTypeEnum = {}));
var IfcCoolingTowerTypeEnum;
(function (IfcCoolingTowerTypeEnum) {
    IfcCoolingTowerTypeEnum["naturaldraft"] = "naturaldraft";
    IfcCoolingTowerTypeEnum["mechanicalinduceddraft"] = "mechanicalinduceddraft";
    IfcCoolingTowerTypeEnum["mechanicalforceddraft"] = "mechanicalforceddraft";
    IfcCoolingTowerTypeEnum["userdefined"] = "userdefined";
    IfcCoolingTowerTypeEnum["notdefined"] = "notdefined";
})(IfcCoolingTowerTypeEnum || (IfcCoolingTowerTypeEnum = {}));
var IfcCostItemTypeEnum;
(function (IfcCostItemTypeEnum) {
    IfcCostItemTypeEnum["userdefined"] = "userdefined";
    IfcCostItemTypeEnum["notdefined"] = "notdefined";
})(IfcCostItemTypeEnum || (IfcCostItemTypeEnum = {}));
var IfcCostScheduleTypeEnum;
(function (IfcCostScheduleTypeEnum) {
    IfcCostScheduleTypeEnum["budget"] = "budget";
    IfcCostScheduleTypeEnum["costplan"] = "costplan";
    IfcCostScheduleTypeEnum["estimate"] = "estimate";
    IfcCostScheduleTypeEnum["tender"] = "tender";
    IfcCostScheduleTypeEnum["pricedbillofquantities"] = "pricedbillofquantities";
    IfcCostScheduleTypeEnum["unpricedbillofquantities"] = "unpricedbillofquantities";
    IfcCostScheduleTypeEnum["scheduleofrates"] = "scheduleofrates";
    IfcCostScheduleTypeEnum["userdefined"] = "userdefined";
    IfcCostScheduleTypeEnum["notdefined"] = "notdefined";
})(IfcCostScheduleTypeEnum || (IfcCostScheduleTypeEnum = {}));
var IfcCoveringTypeEnum;
(function (IfcCoveringTypeEnum) {
    IfcCoveringTypeEnum["ceiling"] = "ceiling";
    IfcCoveringTypeEnum["flooring"] = "flooring";
    IfcCoveringTypeEnum["cladding"] = "cladding";
    IfcCoveringTypeEnum["roofing"] = "roofing";
    IfcCoveringTypeEnum["molding"] = "molding";
    IfcCoveringTypeEnum["skirtingboard"] = "skirtingboard";
    IfcCoveringTypeEnum["insulation"] = "insulation";
    IfcCoveringTypeEnum["membrane"] = "membrane";
    IfcCoveringTypeEnum["sleeving"] = "sleeving";
    IfcCoveringTypeEnum["wrapping"] = "wrapping";
    IfcCoveringTypeEnum["userdefined"] = "userdefined";
    IfcCoveringTypeEnum["notdefined"] = "notdefined";
})(IfcCoveringTypeEnum || (IfcCoveringTypeEnum = {}));
var IfcCrewResourceTypeEnum;
(function (IfcCrewResourceTypeEnum) {
    IfcCrewResourceTypeEnum["office"] = "office";
    IfcCrewResourceTypeEnum["site"] = "site";
    IfcCrewResourceTypeEnum["userdefined"] = "userdefined";
    IfcCrewResourceTypeEnum["notdefined"] = "notdefined";
})(IfcCrewResourceTypeEnum || (IfcCrewResourceTypeEnum = {}));
var IfcCurtainWallTypeEnum;
(function (IfcCurtainWallTypeEnum) {
    IfcCurtainWallTypeEnum["userdefined"] = "userdefined";
    IfcCurtainWallTypeEnum["notdefined"] = "notdefined";
})(IfcCurtainWallTypeEnum || (IfcCurtainWallTypeEnum = {}));
var IfcCurveInterpolationEnum;
(function (IfcCurveInterpolationEnum) {
    IfcCurveInterpolationEnum["linear"] = "linear";
    IfcCurveInterpolationEnum["log_linear"] = "log_linear";
    IfcCurveInterpolationEnum["log_log"] = "log_log";
    IfcCurveInterpolationEnum["notdefined"] = "notdefined";
})(IfcCurveInterpolationEnum || (IfcCurveInterpolationEnum = {}));
var IfcDamperTypeEnum;
(function (IfcDamperTypeEnum) {
    IfcDamperTypeEnum["backdraftdamper"] = "backdraftdamper";
    IfcDamperTypeEnum["balancingdamper"] = "balancingdamper";
    IfcDamperTypeEnum["blastdamper"] = "blastdamper";
    IfcDamperTypeEnum["controldamper"] = "controldamper";
    IfcDamperTypeEnum["firedamper"] = "firedamper";
    IfcDamperTypeEnum["firesmokedamper"] = "firesmokedamper";
    IfcDamperTypeEnum["fumehoodexhaust"] = "fumehoodexhaust";
    IfcDamperTypeEnum["gravitydamper"] = "gravitydamper";
    IfcDamperTypeEnum["gravityreliefdamper"] = "gravityreliefdamper";
    IfcDamperTypeEnum["reliefdamper"] = "reliefdamper";
    IfcDamperTypeEnum["smokedamper"] = "smokedamper";
    IfcDamperTypeEnum["userdefined"] = "userdefined";
    IfcDamperTypeEnum["notdefined"] = "notdefined";
})(IfcDamperTypeEnum || (IfcDamperTypeEnum = {}));
var IfcDataOriginEnum;
(function (IfcDataOriginEnum) {
    IfcDataOriginEnum["measured"] = "measured";
    IfcDataOriginEnum["predicted"] = "predicted";
    IfcDataOriginEnum["simulated"] = "simulated";
    IfcDataOriginEnum["userdefined"] = "userdefined";
    IfcDataOriginEnum["notdefined"] = "notdefined";
})(IfcDataOriginEnum || (IfcDataOriginEnum = {}));
var IfcDerivedUnitEnum;
(function (IfcDerivedUnitEnum) {
    IfcDerivedUnitEnum["angularvelocityunit"] = "angularvelocityunit";
    IfcDerivedUnitEnum["areadensityunit"] = "areadensityunit";
    IfcDerivedUnitEnum["compoundplaneangleunit"] = "compoundplaneangleunit";
    IfcDerivedUnitEnum["dynamicviscosityunit"] = "dynamicviscosityunit";
    IfcDerivedUnitEnum["heatfluxdensityunit"] = "heatfluxdensityunit";
    IfcDerivedUnitEnum["integercountrateunit"] = "integercountrateunit";
    IfcDerivedUnitEnum["isothermalmoisturecapacityunit"] = "isothermalmoisturecapacityunit";
    IfcDerivedUnitEnum["kinematicviscosityunit"] = "kinematicviscosityunit";
    IfcDerivedUnitEnum["linearvelocityunit"] = "linearvelocityunit";
    IfcDerivedUnitEnum["massdensityunit"] = "massdensityunit";
    IfcDerivedUnitEnum["massflowrateunit"] = "massflowrateunit";
    IfcDerivedUnitEnum["moisturediffusivityunit"] = "moisturediffusivityunit";
    IfcDerivedUnitEnum["molecularweightunit"] = "molecularweightunit";
    IfcDerivedUnitEnum["specificheatcapacityunit"] = "specificheatcapacityunit";
    IfcDerivedUnitEnum["thermaladmittanceunit"] = "thermaladmittanceunit";
    IfcDerivedUnitEnum["thermalconductanceunit"] = "thermalconductanceunit";
    IfcDerivedUnitEnum["thermalresistanceunit"] = "thermalresistanceunit";
    IfcDerivedUnitEnum["thermaltransmittanceunit"] = "thermaltransmittanceunit";
    IfcDerivedUnitEnum["vaporpermeabilityunit"] = "vaporpermeabilityunit";
    IfcDerivedUnitEnum["volumetricflowrateunit"] = "volumetricflowrateunit";
    IfcDerivedUnitEnum["rotationalfrequencyunit"] = "rotationalfrequencyunit";
    IfcDerivedUnitEnum["torqueunit"] = "torqueunit";
    IfcDerivedUnitEnum["momentofinertiaunit"] = "momentofinertiaunit";
    IfcDerivedUnitEnum["linearmomentunit"] = "linearmomentunit";
    IfcDerivedUnitEnum["linearforceunit"] = "linearforceunit";
    IfcDerivedUnitEnum["planarforceunit"] = "planarforceunit";
    IfcDerivedUnitEnum["modulusofelasticityunit"] = "modulusofelasticityunit";
    IfcDerivedUnitEnum["shearmodulusunit"] = "shearmodulusunit";
    IfcDerivedUnitEnum["linearstiffnessunit"] = "linearstiffnessunit";
    IfcDerivedUnitEnum["rotationalstiffnessunit"] = "rotationalstiffnessunit";
    IfcDerivedUnitEnum["modulusofsubgradereactionunit"] = "modulusofsubgradereactionunit";
    IfcDerivedUnitEnum["accelerationunit"] = "accelerationunit";
    IfcDerivedUnitEnum["curvatureunit"] = "curvatureunit";
    IfcDerivedUnitEnum["heatingvalueunit"] = "heatingvalueunit";
    IfcDerivedUnitEnum["ionconcentrationunit"] = "ionconcentrationunit";
    IfcDerivedUnitEnum["luminousintensitydistributionunit"] = "luminousintensitydistributionunit";
    IfcDerivedUnitEnum["massperlengthunit"] = "massperlengthunit";
    IfcDerivedUnitEnum["modulusoflinearsubgradereactionunit"] = "modulusoflinearsubgradereactionunit";
    IfcDerivedUnitEnum["modulusofrotationalsubgradereactionunit"] = "modulusofrotationalsubgradereactionunit";
    IfcDerivedUnitEnum["phunit"] = "phunit";
    IfcDerivedUnitEnum["rotationalmassunit"] = "rotationalmassunit";
    IfcDerivedUnitEnum["sectionareaintegralunit"] = "sectionareaintegralunit";
    IfcDerivedUnitEnum["sectionmodulusunit"] = "sectionmodulusunit";
    IfcDerivedUnitEnum["soundpowerlevelunit"] = "soundpowerlevelunit";
    IfcDerivedUnitEnum["soundpowerunit"] = "soundpowerunit";
    IfcDerivedUnitEnum["soundpressurelevelunit"] = "soundpressurelevelunit";
    IfcDerivedUnitEnum["soundpressureunit"] = "soundpressureunit";
    IfcDerivedUnitEnum["temperaturegradientunit"] = "temperaturegradientunit";
    IfcDerivedUnitEnum["temperaturerateofchangeunit"] = "temperaturerateofchangeunit";
    IfcDerivedUnitEnum["thermalexpansioncoefficientunit"] = "thermalexpansioncoefficientunit";
    IfcDerivedUnitEnum["warpingconstantunit"] = "warpingconstantunit";
    IfcDerivedUnitEnum["warpingmomentunit"] = "warpingmomentunit";
    IfcDerivedUnitEnum["userdefined"] = "userdefined";
})(IfcDerivedUnitEnum || (IfcDerivedUnitEnum = {}));
var IfcDirectionSenseEnum;
(function (IfcDirectionSenseEnum) {
    IfcDirectionSenseEnum["positive"] = "positive";
    IfcDirectionSenseEnum["negative"] = "negative";
})(IfcDirectionSenseEnum || (IfcDirectionSenseEnum = {}));
var IfcDiscreteAccessoryTypeEnum;
(function (IfcDiscreteAccessoryTypeEnum) {
    IfcDiscreteAccessoryTypeEnum["anchorplate"] = "anchorplate";
    IfcDiscreteAccessoryTypeEnum["bracket"] = "bracket";
    IfcDiscreteAccessoryTypeEnum["shoe"] = "shoe";
    IfcDiscreteAccessoryTypeEnum["userdefined"] = "userdefined";
    IfcDiscreteAccessoryTypeEnum["notdefined"] = "notdefined";
})(IfcDiscreteAccessoryTypeEnum || (IfcDiscreteAccessoryTypeEnum = {}));
var IfcDistributionChamberElementTypeEnum;
(function (IfcDistributionChamberElementTypeEnum) {
    IfcDistributionChamberElementTypeEnum["formedduct"] = "formedduct";
    IfcDistributionChamberElementTypeEnum["inspectionchamber"] = "inspectionchamber";
    IfcDistributionChamberElementTypeEnum["inspectionpit"] = "inspectionpit";
    IfcDistributionChamberElementTypeEnum["manhole"] = "manhole";
    IfcDistributionChamberElementTypeEnum["meterchamber"] = "meterchamber";
    IfcDistributionChamberElementTypeEnum["sump"] = "sump";
    IfcDistributionChamberElementTypeEnum["trench"] = "trench";
    IfcDistributionChamberElementTypeEnum["valvechamber"] = "valvechamber";
    IfcDistributionChamberElementTypeEnum["userdefined"] = "userdefined";
    IfcDistributionChamberElementTypeEnum["notdefined"] = "notdefined";
})(IfcDistributionChamberElementTypeEnum || (IfcDistributionChamberElementTypeEnum = {}));
var IfcDistributionPortTypeEnum;
(function (IfcDistributionPortTypeEnum) {
    IfcDistributionPortTypeEnum["cable"] = "cable";
    IfcDistributionPortTypeEnum["cablecarrier"] = "cablecarrier";
    IfcDistributionPortTypeEnum["duct"] = "duct";
    IfcDistributionPortTypeEnum["pipe"] = "pipe";
    IfcDistributionPortTypeEnum["userdefined"] = "userdefined";
    IfcDistributionPortTypeEnum["notdefined"] = "notdefined";
})(IfcDistributionPortTypeEnum || (IfcDistributionPortTypeEnum = {}));
var IfcDistributionSystemEnum;
(function (IfcDistributionSystemEnum) {
    IfcDistributionSystemEnum["airconditioning"] = "airconditioning";
    IfcDistributionSystemEnum["audiovisual"] = "audiovisual";
    IfcDistributionSystemEnum["chemical"] = "chemical";
    IfcDistributionSystemEnum["chilledwater"] = "chilledwater";
    IfcDistributionSystemEnum["communication"] = "communication";
    IfcDistributionSystemEnum["compressedair"] = "compressedair";
    IfcDistributionSystemEnum["condenserwater"] = "condenserwater";
    IfcDistributionSystemEnum["control"] = "control";
    IfcDistributionSystemEnum["conveying"] = "conveying";
    IfcDistributionSystemEnum["data"] = "data";
    IfcDistributionSystemEnum["disposal"] = "disposal";
    IfcDistributionSystemEnum["domesticcoldwater"] = "domesticcoldwater";
    IfcDistributionSystemEnum["domestichotwater"] = "domestichotwater";
    IfcDistributionSystemEnum["drainage"] = "drainage";
    IfcDistributionSystemEnum["earthing"] = "earthing";
    IfcDistributionSystemEnum["electrical"] = "electrical";
    IfcDistributionSystemEnum["electroacoustic"] = "electroacoustic";
    IfcDistributionSystemEnum["exhaust"] = "exhaust";
    IfcDistributionSystemEnum["fireprotection"] = "fireprotection";
    IfcDistributionSystemEnum["fuel"] = "fuel";
    IfcDistributionSystemEnum["gas"] = "gas";
    IfcDistributionSystemEnum["hazardous"] = "hazardous";
    IfcDistributionSystemEnum["heating"] = "heating";
    IfcDistributionSystemEnum["lighting"] = "lighting";
    IfcDistributionSystemEnum["lightningprotection"] = "lightningprotection";
    IfcDistributionSystemEnum["municipalsolidwaste"] = "municipalsolidwaste";
    IfcDistributionSystemEnum["oil"] = "oil";
    IfcDistributionSystemEnum["operational"] = "operational";
    IfcDistributionSystemEnum["powergeneration"] = "powergeneration";
    IfcDistributionSystemEnum["rainwater"] = "rainwater";
    IfcDistributionSystemEnum["refrigeration"] = "refrigeration";
    IfcDistributionSystemEnum["security"] = "security";
    IfcDistributionSystemEnum["sewage"] = "sewage";
    IfcDistributionSystemEnum["signal"] = "signal";
    IfcDistributionSystemEnum["stormwater"] = "stormwater";
    IfcDistributionSystemEnum["telephone"] = "telephone";
    IfcDistributionSystemEnum["tv"] = "tv";
    IfcDistributionSystemEnum["vacuum"] = "vacuum";
    IfcDistributionSystemEnum["vent"] = "vent";
    IfcDistributionSystemEnum["ventilation"] = "ventilation";
    IfcDistributionSystemEnum["wastewater"] = "wastewater";
    IfcDistributionSystemEnum["watersupply"] = "watersupply";
    IfcDistributionSystemEnum["userdefined"] = "userdefined";
    IfcDistributionSystemEnum["notdefined"] = "notdefined";
})(IfcDistributionSystemEnum || (IfcDistributionSystemEnum = {}));
var IfcDocumentConfidentialityEnum;
(function (IfcDocumentConfidentialityEnum) {
    IfcDocumentConfidentialityEnum["public"] = "public";
    IfcDocumentConfidentialityEnum["restricted"] = "restricted";
    IfcDocumentConfidentialityEnum["confidential"] = "confidential";
    IfcDocumentConfidentialityEnum["personal"] = "personal";
    IfcDocumentConfidentialityEnum["userdefined"] = "userdefined";
    IfcDocumentConfidentialityEnum["notdefined"] = "notdefined";
})(IfcDocumentConfidentialityEnum || (IfcDocumentConfidentialityEnum = {}));
var IfcDocumentStatusEnum;
(function (IfcDocumentStatusEnum) {
    IfcDocumentStatusEnum["draft"] = "draft";
    IfcDocumentStatusEnum["finaldraft"] = "finaldraft";
    IfcDocumentStatusEnum["final"] = "final";
    IfcDocumentStatusEnum["revision"] = "revision";
    IfcDocumentStatusEnum["notdefined"] = "notdefined";
})(IfcDocumentStatusEnum || (IfcDocumentStatusEnum = {}));
var IfcDoorPanelOperationEnum;
(function (IfcDoorPanelOperationEnum) {
    IfcDoorPanelOperationEnum["swinging"] = "swinging";
    IfcDoorPanelOperationEnum["double_acting"] = "double_acting";
    IfcDoorPanelOperationEnum["sliding"] = "sliding";
    IfcDoorPanelOperationEnum["folding"] = "folding";
    IfcDoorPanelOperationEnum["revolving"] = "revolving";
    IfcDoorPanelOperationEnum["rollingup"] = "rollingup";
    IfcDoorPanelOperationEnum["fixedpanel"] = "fixedpanel";
    IfcDoorPanelOperationEnum["userdefined"] = "userdefined";
    IfcDoorPanelOperationEnum["notdefined"] = "notdefined";
})(IfcDoorPanelOperationEnum || (IfcDoorPanelOperationEnum = {}));
var IfcDoorPanelPositionEnum;
(function (IfcDoorPanelPositionEnum) {
    IfcDoorPanelPositionEnum["left"] = "left";
    IfcDoorPanelPositionEnum["middle"] = "middle";
    IfcDoorPanelPositionEnum["right"] = "right";
    IfcDoorPanelPositionEnum["notdefined"] = "notdefined";
})(IfcDoorPanelPositionEnum || (IfcDoorPanelPositionEnum = {}));
var IfcDoorStyleConstructionEnum;
(function (IfcDoorStyleConstructionEnum) {
    IfcDoorStyleConstructionEnum["aluminium"] = "aluminium";
    IfcDoorStyleConstructionEnum["high_grade_steel"] = "high_grade_steel";
    IfcDoorStyleConstructionEnum["steel"] = "steel";
    IfcDoorStyleConstructionEnum["wood"] = "wood";
    IfcDoorStyleConstructionEnum["aluminium_wood"] = "aluminium_wood";
    IfcDoorStyleConstructionEnum["aluminium_plastic"] = "aluminium_plastic";
    IfcDoorStyleConstructionEnum["plastic"] = "plastic";
    IfcDoorStyleConstructionEnum["userdefined"] = "userdefined";
    IfcDoorStyleConstructionEnum["notdefined"] = "notdefined";
})(IfcDoorStyleConstructionEnum || (IfcDoorStyleConstructionEnum = {}));
var IfcDoorStyleOperationEnum;
(function (IfcDoorStyleOperationEnum) {
    IfcDoorStyleOperationEnum["single_swing_left"] = "single_swing_left";
    IfcDoorStyleOperationEnum["single_swing_right"] = "single_swing_right";
    IfcDoorStyleOperationEnum["double_door_single_swing"] = "double_door_single_swing";
    IfcDoorStyleOperationEnum["double_door_single_swing_opposite_left"] = "double_door_single_swing_opposite_left";
    IfcDoorStyleOperationEnum["double_door_single_swing_opposite_right"] = "double_door_single_swing_opposite_right";
    IfcDoorStyleOperationEnum["double_swing_left"] = "double_swing_left";
    IfcDoorStyleOperationEnum["double_swing_right"] = "double_swing_right";
    IfcDoorStyleOperationEnum["double_door_double_swing"] = "double_door_double_swing";
    IfcDoorStyleOperationEnum["sliding_to_left"] = "sliding_to_left";
    IfcDoorStyleOperationEnum["sliding_to_right"] = "sliding_to_right";
    IfcDoorStyleOperationEnum["double_door_sliding"] = "double_door_sliding";
    IfcDoorStyleOperationEnum["folding_to_left"] = "folding_to_left";
    IfcDoorStyleOperationEnum["folding_to_right"] = "folding_to_right";
    IfcDoorStyleOperationEnum["double_door_folding"] = "double_door_folding";
    IfcDoorStyleOperationEnum["revolving"] = "revolving";
    IfcDoorStyleOperationEnum["rollingup"] = "rollingup";
    IfcDoorStyleOperationEnum["userdefined"] = "userdefined";
    IfcDoorStyleOperationEnum["notdefined"] = "notdefined";
})(IfcDoorStyleOperationEnum || (IfcDoorStyleOperationEnum = {}));
var IfcDoorTypeEnum;
(function (IfcDoorTypeEnum) {
    IfcDoorTypeEnum["door"] = "door";
    IfcDoorTypeEnum["gate"] = "gate";
    IfcDoorTypeEnum["trapdoor"] = "trapdoor";
    IfcDoorTypeEnum["userdefined"] = "userdefined";
    IfcDoorTypeEnum["notdefined"] = "notdefined";
})(IfcDoorTypeEnum || (IfcDoorTypeEnum = {}));
var IfcDoorTypeOperationEnum;
(function (IfcDoorTypeOperationEnum) {
    IfcDoorTypeOperationEnum["single_swing_left"] = "single_swing_left";
    IfcDoorTypeOperationEnum["single_swing_right"] = "single_swing_right";
    IfcDoorTypeOperationEnum["double_door_single_swing"] = "double_door_single_swing";
    IfcDoorTypeOperationEnum["double_door_single_swing_opposite_left"] = "double_door_single_swing_opposite_left";
    IfcDoorTypeOperationEnum["double_door_single_swing_opposite_right"] = "double_door_single_swing_opposite_right";
    IfcDoorTypeOperationEnum["double_swing_left"] = "double_swing_left";
    IfcDoorTypeOperationEnum["double_swing_right"] = "double_swing_right";
    IfcDoorTypeOperationEnum["double_door_double_swing"] = "double_door_double_swing";
    IfcDoorTypeOperationEnum["sliding_to_left"] = "sliding_to_left";
    IfcDoorTypeOperationEnum["sliding_to_right"] = "sliding_to_right";
    IfcDoorTypeOperationEnum["double_door_sliding"] = "double_door_sliding";
    IfcDoorTypeOperationEnum["folding_to_left"] = "folding_to_left";
    IfcDoorTypeOperationEnum["folding_to_right"] = "folding_to_right";
    IfcDoorTypeOperationEnum["double_door_folding"] = "double_door_folding";
    IfcDoorTypeOperationEnum["revolving"] = "revolving";
    IfcDoorTypeOperationEnum["rollingup"] = "rollingup";
    IfcDoorTypeOperationEnum["swing_fixed_left"] = "swing_fixed_left";
    IfcDoorTypeOperationEnum["swing_fixed_right"] = "swing_fixed_right";
    IfcDoorTypeOperationEnum["userdefined"] = "userdefined";
    IfcDoorTypeOperationEnum["notdefined"] = "notdefined";
})(IfcDoorTypeOperationEnum || (IfcDoorTypeOperationEnum = {}));
var IfcDuctFittingTypeEnum;
(function (IfcDuctFittingTypeEnum) {
    IfcDuctFittingTypeEnum["bend"] = "bend";
    IfcDuctFittingTypeEnum["connector"] = "connector";
    IfcDuctFittingTypeEnum["entry"] = "entry";
    IfcDuctFittingTypeEnum["exit"] = "exit";
    IfcDuctFittingTypeEnum["junction"] = "junction";
    IfcDuctFittingTypeEnum["obstruction"] = "obstruction";
    IfcDuctFittingTypeEnum["transition"] = "transition";
    IfcDuctFittingTypeEnum["userdefined"] = "userdefined";
    IfcDuctFittingTypeEnum["notdefined"] = "notdefined";
})(IfcDuctFittingTypeEnum || (IfcDuctFittingTypeEnum = {}));
var IfcDuctSegmentTypeEnum;
(function (IfcDuctSegmentTypeEnum) {
    IfcDuctSegmentTypeEnum["rigidsegment"] = "rigidsegment";
    IfcDuctSegmentTypeEnum["flexiblesegment"] = "flexiblesegment";
    IfcDuctSegmentTypeEnum["userdefined"] = "userdefined";
    IfcDuctSegmentTypeEnum["notdefined"] = "notdefined";
})(IfcDuctSegmentTypeEnum || (IfcDuctSegmentTypeEnum = {}));
var IfcDuctSilencerTypeEnum;
(function (IfcDuctSilencerTypeEnum) {
    IfcDuctSilencerTypeEnum["flatoval"] = "flatoval";
    IfcDuctSilencerTypeEnum["rectangular"] = "rectangular";
    IfcDuctSilencerTypeEnum["round"] = "round";
    IfcDuctSilencerTypeEnum["userdefined"] = "userdefined";
    IfcDuctSilencerTypeEnum["notdefined"] = "notdefined";
})(IfcDuctSilencerTypeEnum || (IfcDuctSilencerTypeEnum = {}));
var IfcElectricApplianceTypeEnum;
(function (IfcElectricApplianceTypeEnum) {
    IfcElectricApplianceTypeEnum["dishwasher"] = "dishwasher";
    IfcElectricApplianceTypeEnum["electriccooker"] = "electriccooker";
    IfcElectricApplianceTypeEnum["freestandingelectricheater"] = "freestandingelectricheater";
    IfcElectricApplianceTypeEnum["freestandingfan"] = "freestandingfan";
    IfcElectricApplianceTypeEnum["freestandingwaterheater"] = "freestandingwaterheater";
    IfcElectricApplianceTypeEnum["freestandingwatercooler"] = "freestandingwatercooler";
    IfcElectricApplianceTypeEnum["freezer"] = "freezer";
    IfcElectricApplianceTypeEnum["fridge_freezer"] = "fridge_freezer";
    IfcElectricApplianceTypeEnum["handdryer"] = "handdryer";
    IfcElectricApplianceTypeEnum["kitchenmachine"] = "kitchenmachine";
    IfcElectricApplianceTypeEnum["microwave"] = "microwave";
    IfcElectricApplianceTypeEnum["photocopier"] = "photocopier";
    IfcElectricApplianceTypeEnum["refrigerator"] = "refrigerator";
    IfcElectricApplianceTypeEnum["tumbledryer"] = "tumbledryer";
    IfcElectricApplianceTypeEnum["vendingmachine"] = "vendingmachine";
    IfcElectricApplianceTypeEnum["washingmachine"] = "washingmachine";
    IfcElectricApplianceTypeEnum["userdefined"] = "userdefined";
    IfcElectricApplianceTypeEnum["notdefined"] = "notdefined";
})(IfcElectricApplianceTypeEnum || (IfcElectricApplianceTypeEnum = {}));
var IfcElectricDistributionBoardTypeEnum;
(function (IfcElectricDistributionBoardTypeEnum) {
    IfcElectricDistributionBoardTypeEnum["consumerunit"] = "consumerunit";
    IfcElectricDistributionBoardTypeEnum["distributionboard"] = "distributionboard";
    IfcElectricDistributionBoardTypeEnum["motorcontrolcentre"] = "motorcontrolcentre";
    IfcElectricDistributionBoardTypeEnum["switchboard"] = "switchboard";
    IfcElectricDistributionBoardTypeEnum["userdefined"] = "userdefined";
    IfcElectricDistributionBoardTypeEnum["notdefined"] = "notdefined";
})(IfcElectricDistributionBoardTypeEnum || (IfcElectricDistributionBoardTypeEnum = {}));
var IfcElectricFlowStorageDeviceTypeEnum;
(function (IfcElectricFlowStorageDeviceTypeEnum) {
    IfcElectricFlowStorageDeviceTypeEnum["battery"] = "battery";
    IfcElectricFlowStorageDeviceTypeEnum["capacitorbank"] = "capacitorbank";
    IfcElectricFlowStorageDeviceTypeEnum["harmonicfilter"] = "harmonicfilter";
    IfcElectricFlowStorageDeviceTypeEnum["inductorbank"] = "inductorbank";
    IfcElectricFlowStorageDeviceTypeEnum["ups"] = "ups";
    IfcElectricFlowStorageDeviceTypeEnum["userdefined"] = "userdefined";
    IfcElectricFlowStorageDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcElectricFlowStorageDeviceTypeEnum || (IfcElectricFlowStorageDeviceTypeEnum = {}));
var IfcElectricGeneratorTypeEnum;
(function (IfcElectricGeneratorTypeEnum) {
    IfcElectricGeneratorTypeEnum["chp"] = "chp";
    IfcElectricGeneratorTypeEnum["enginegenerator"] = "enginegenerator";
    IfcElectricGeneratorTypeEnum["standalone"] = "standalone";
    IfcElectricGeneratorTypeEnum["userdefined"] = "userdefined";
    IfcElectricGeneratorTypeEnum["notdefined"] = "notdefined";
})(IfcElectricGeneratorTypeEnum || (IfcElectricGeneratorTypeEnum = {}));
var IfcElectricMotorTypeEnum;
(function (IfcElectricMotorTypeEnum) {
    IfcElectricMotorTypeEnum["dc"] = "dc";
    IfcElectricMotorTypeEnum["induction"] = "induction";
    IfcElectricMotorTypeEnum["polyphase"] = "polyphase";
    IfcElectricMotorTypeEnum["reluctancesynchronous"] = "reluctancesynchronous";
    IfcElectricMotorTypeEnum["synchronous"] = "synchronous";
    IfcElectricMotorTypeEnum["userdefined"] = "userdefined";
    IfcElectricMotorTypeEnum["notdefined"] = "notdefined";
})(IfcElectricMotorTypeEnum || (IfcElectricMotorTypeEnum = {}));
var IfcElectricTimeControlTypeEnum;
(function (IfcElectricTimeControlTypeEnum) {
    IfcElectricTimeControlTypeEnum["timeclock"] = "timeclock";
    IfcElectricTimeControlTypeEnum["timedelay"] = "timedelay";
    IfcElectricTimeControlTypeEnum["relay"] = "relay";
    IfcElectricTimeControlTypeEnum["userdefined"] = "userdefined";
    IfcElectricTimeControlTypeEnum["notdefined"] = "notdefined";
})(IfcElectricTimeControlTypeEnum || (IfcElectricTimeControlTypeEnum = {}));
var IfcElementAssemblyTypeEnum;
(function (IfcElementAssemblyTypeEnum) {
    IfcElementAssemblyTypeEnum["accessory_assembly"] = "accessory_assembly";
    IfcElementAssemblyTypeEnum["arch"] = "arch";
    IfcElementAssemblyTypeEnum["beam_grid"] = "beam_grid";
    IfcElementAssemblyTypeEnum["braced_frame"] = "braced_frame";
    IfcElementAssemblyTypeEnum["girder"] = "girder";
    IfcElementAssemblyTypeEnum["reinforcement_unit"] = "reinforcement_unit";
    IfcElementAssemblyTypeEnum["rigid_frame"] = "rigid_frame";
    IfcElementAssemblyTypeEnum["slab_field"] = "slab_field";
    IfcElementAssemblyTypeEnum["truss"] = "truss";
    IfcElementAssemblyTypeEnum["userdefined"] = "userdefined";
    IfcElementAssemblyTypeEnum["notdefined"] = "notdefined";
})(IfcElementAssemblyTypeEnum || (IfcElementAssemblyTypeEnum = {}));
var IfcElementCompositionEnum;
(function (IfcElementCompositionEnum) {
    IfcElementCompositionEnum["complex"] = "complex";
    IfcElementCompositionEnum["element"] = "element";
    IfcElementCompositionEnum["partial"] = "partial";
})(IfcElementCompositionEnum || (IfcElementCompositionEnum = {}));
var IfcEngineTypeEnum;
(function (IfcEngineTypeEnum) {
    IfcEngineTypeEnum["externalcombustion"] = "externalcombustion";
    IfcEngineTypeEnum["internalcombustion"] = "internalcombustion";
    IfcEngineTypeEnum["userdefined"] = "userdefined";
    IfcEngineTypeEnum["notdefined"] = "notdefined";
})(IfcEngineTypeEnum || (IfcEngineTypeEnum = {}));
var IfcEvaporativeCoolerTypeEnum;
(function (IfcEvaporativeCoolerTypeEnum) {
    IfcEvaporativeCoolerTypeEnum["directevaporativerandommediaaircooler"] = "directevaporativerandommediaaircooler";
    IfcEvaporativeCoolerTypeEnum["directevaporativerigidmediaaircooler"] = "directevaporativerigidmediaaircooler";
    IfcEvaporativeCoolerTypeEnum["directevaporativeslingerspackagedaircooler"] = "directevaporativeslingerspackagedaircooler";
    IfcEvaporativeCoolerTypeEnum["directevaporativepackagedrotaryaircooler"] = "directevaporativepackagedrotaryaircooler";
    IfcEvaporativeCoolerTypeEnum["directevaporativeairwasher"] = "directevaporativeairwasher";
    IfcEvaporativeCoolerTypeEnum["indirectevaporativepackageaircooler"] = "indirectevaporativepackageaircooler";
    IfcEvaporativeCoolerTypeEnum["indirectevaporativewetcoil"] = "indirectevaporativewetcoil";
    IfcEvaporativeCoolerTypeEnum["indirectevaporativecoolingtowerorcoilcooler"] = "indirectevaporativecoolingtowerorcoilcooler";
    IfcEvaporativeCoolerTypeEnum["indirectdirectcombination"] = "indirectdirectcombination";
    IfcEvaporativeCoolerTypeEnum["userdefined"] = "userdefined";
    IfcEvaporativeCoolerTypeEnum["notdefined"] = "notdefined";
})(IfcEvaporativeCoolerTypeEnum || (IfcEvaporativeCoolerTypeEnum = {}));
var IfcEvaporatorTypeEnum;
(function (IfcEvaporatorTypeEnum) {
    IfcEvaporatorTypeEnum["directexpansion"] = "directexpansion";
    IfcEvaporatorTypeEnum["directexpansionshellandtube"] = "directexpansionshellandtube";
    IfcEvaporatorTypeEnum["directexpansiontubeintube"] = "directexpansiontubeintube";
    IfcEvaporatorTypeEnum["directexpansionbrazedplate"] = "directexpansionbrazedplate";
    IfcEvaporatorTypeEnum["floodedshellandtube"] = "floodedshellandtube";
    IfcEvaporatorTypeEnum["shellandcoil"] = "shellandcoil";
    IfcEvaporatorTypeEnum["userdefined"] = "userdefined";
    IfcEvaporatorTypeEnum["notdefined"] = "notdefined";
})(IfcEvaporatorTypeEnum || (IfcEvaporatorTypeEnum = {}));
var IfcEventTriggerTypeEnum;
(function (IfcEventTriggerTypeEnum) {
    IfcEventTriggerTypeEnum["eventrule"] = "eventrule";
    IfcEventTriggerTypeEnum["eventmessage"] = "eventmessage";
    IfcEventTriggerTypeEnum["eventtime"] = "eventtime";
    IfcEventTriggerTypeEnum["eventcomplex"] = "eventcomplex";
    IfcEventTriggerTypeEnum["userdefined"] = "userdefined";
    IfcEventTriggerTypeEnum["notdefined"] = "notdefined";
})(IfcEventTriggerTypeEnum || (IfcEventTriggerTypeEnum = {}));
var IfcEventTypeEnum;
(function (IfcEventTypeEnum) {
    IfcEventTypeEnum["startevent"] = "startevent";
    IfcEventTypeEnum["endevent"] = "endevent";
    IfcEventTypeEnum["intermediateevent"] = "intermediateevent";
    IfcEventTypeEnum["userdefined"] = "userdefined";
    IfcEventTypeEnum["notdefined"] = "notdefined";
})(IfcEventTypeEnum || (IfcEventTypeEnum = {}));
var IfcExternalSpatialElementTypeEnum;
(function (IfcExternalSpatialElementTypeEnum) {
    IfcExternalSpatialElementTypeEnum["external"] = "external";
    IfcExternalSpatialElementTypeEnum["external_earth"] = "external_earth";
    IfcExternalSpatialElementTypeEnum["external_water"] = "external_water";
    IfcExternalSpatialElementTypeEnum["external_fire"] = "external_fire";
    IfcExternalSpatialElementTypeEnum["userdefined"] = "userdefined";
    IfcExternalSpatialElementTypeEnum["notdefined"] = "notdefined";
})(IfcExternalSpatialElementTypeEnum || (IfcExternalSpatialElementTypeEnum = {}));
var IfcFanTypeEnum;
(function (IfcFanTypeEnum) {
    IfcFanTypeEnum["centrifugalforwardcurved"] = "centrifugalforwardcurved";
    IfcFanTypeEnum["centrifugalradial"] = "centrifugalradial";
    IfcFanTypeEnum["centrifugalbackwardinclinedcurved"] = "centrifugalbackwardinclinedcurved";
    IfcFanTypeEnum["centrifugalairfoil"] = "centrifugalairfoil";
    IfcFanTypeEnum["tubeaxial"] = "tubeaxial";
    IfcFanTypeEnum["vaneaxial"] = "vaneaxial";
    IfcFanTypeEnum["propelloraxial"] = "propelloraxial";
    IfcFanTypeEnum["userdefined"] = "userdefined";
    IfcFanTypeEnum["notdefined"] = "notdefined";
})(IfcFanTypeEnum || (IfcFanTypeEnum = {}));
var IfcFastenerTypeEnum;
(function (IfcFastenerTypeEnum) {
    IfcFastenerTypeEnum["glue"] = "glue";
    IfcFastenerTypeEnum["mortar"] = "mortar";
    IfcFastenerTypeEnum["weld"] = "weld";
    IfcFastenerTypeEnum["userdefined"] = "userdefined";
    IfcFastenerTypeEnum["notdefined"] = "notdefined";
})(IfcFastenerTypeEnum || (IfcFastenerTypeEnum = {}));
var IfcFilterTypeEnum;
(function (IfcFilterTypeEnum) {
    IfcFilterTypeEnum["airparticlefilter"] = "airparticlefilter";
    IfcFilterTypeEnum["compressedairfilter"] = "compressedairfilter";
    IfcFilterTypeEnum["odorfilter"] = "odorfilter";
    IfcFilterTypeEnum["oilfilter"] = "oilfilter";
    IfcFilterTypeEnum["strainer"] = "strainer";
    IfcFilterTypeEnum["waterfilter"] = "waterfilter";
    IfcFilterTypeEnum["userdefined"] = "userdefined";
    IfcFilterTypeEnum["notdefined"] = "notdefined";
})(IfcFilterTypeEnum || (IfcFilterTypeEnum = {}));
var IfcFireSuppressionTerminalTypeEnum;
(function (IfcFireSuppressionTerminalTypeEnum) {
    IfcFireSuppressionTerminalTypeEnum["breechinginlet"] = "breechinginlet";
    IfcFireSuppressionTerminalTypeEnum["firehydrant"] = "firehydrant";
    IfcFireSuppressionTerminalTypeEnum["hosereel"] = "hosereel";
    IfcFireSuppressionTerminalTypeEnum["sprinkler"] = "sprinkler";
    IfcFireSuppressionTerminalTypeEnum["sprinklerdeflector"] = "sprinklerdeflector";
    IfcFireSuppressionTerminalTypeEnum["userdefined"] = "userdefined";
    IfcFireSuppressionTerminalTypeEnum["notdefined"] = "notdefined";
})(IfcFireSuppressionTerminalTypeEnum || (IfcFireSuppressionTerminalTypeEnum = {}));
var IfcFlowDirectionEnum;
(function (IfcFlowDirectionEnum) {
    IfcFlowDirectionEnum["source"] = "source";
    IfcFlowDirectionEnum["sink"] = "sink";
    IfcFlowDirectionEnum["sourceandsink"] = "sourceandsink";
    IfcFlowDirectionEnum["notdefined"] = "notdefined";
})(IfcFlowDirectionEnum || (IfcFlowDirectionEnum = {}));
var IfcFlowInstrumentTypeEnum;
(function (IfcFlowInstrumentTypeEnum) {
    IfcFlowInstrumentTypeEnum["pressuregauge"] = "pressuregauge";
    IfcFlowInstrumentTypeEnum["thermometer"] = "thermometer";
    IfcFlowInstrumentTypeEnum["ammeter"] = "ammeter";
    IfcFlowInstrumentTypeEnum["frequencymeter"] = "frequencymeter";
    IfcFlowInstrumentTypeEnum["powerfactormeter"] = "powerfactormeter";
    IfcFlowInstrumentTypeEnum["phaseanglemeter"] = "phaseanglemeter";
    IfcFlowInstrumentTypeEnum["voltmeter_peak"] = "voltmeter_peak";
    IfcFlowInstrumentTypeEnum["voltmeter_rms"] = "voltmeter_rms";
    IfcFlowInstrumentTypeEnum["userdefined"] = "userdefined";
    IfcFlowInstrumentTypeEnum["notdefined"] = "notdefined";
})(IfcFlowInstrumentTypeEnum || (IfcFlowInstrumentTypeEnum = {}));
var IfcFlowMeterTypeEnum;
(function (IfcFlowMeterTypeEnum) {
    IfcFlowMeterTypeEnum["energymeter"] = "energymeter";
    IfcFlowMeterTypeEnum["gasmeter"] = "gasmeter";
    IfcFlowMeterTypeEnum["oilmeter"] = "oilmeter";
    IfcFlowMeterTypeEnum["watermeter"] = "watermeter";
    IfcFlowMeterTypeEnum["userdefined"] = "userdefined";
    IfcFlowMeterTypeEnum["notdefined"] = "notdefined";
})(IfcFlowMeterTypeEnum || (IfcFlowMeterTypeEnum = {}));
var IfcFootingTypeEnum;
(function (IfcFootingTypeEnum) {
    IfcFootingTypeEnum["caisson_foundation"] = "caisson_foundation";
    IfcFootingTypeEnum["footing_beam"] = "footing_beam";
    IfcFootingTypeEnum["pad_footing"] = "pad_footing";
    IfcFootingTypeEnum["pile_cap"] = "pile_cap";
    IfcFootingTypeEnum["strip_footing"] = "strip_footing";
    IfcFootingTypeEnum["userdefined"] = "userdefined";
    IfcFootingTypeEnum["notdefined"] = "notdefined";
})(IfcFootingTypeEnum || (IfcFootingTypeEnum = {}));
var IfcFurnitureTypeEnum;
(function (IfcFurnitureTypeEnum) {
    IfcFurnitureTypeEnum["chair"] = "chair";
    IfcFurnitureTypeEnum["table"] = "table";
    IfcFurnitureTypeEnum["desk"] = "desk";
    IfcFurnitureTypeEnum["bed"] = "bed";
    IfcFurnitureTypeEnum["filecabinet"] = "filecabinet";
    IfcFurnitureTypeEnum["shelf"] = "shelf";
    IfcFurnitureTypeEnum["sofa"] = "sofa";
    IfcFurnitureTypeEnum["userdefined"] = "userdefined";
    IfcFurnitureTypeEnum["notdefined"] = "notdefined";
})(IfcFurnitureTypeEnum || (IfcFurnitureTypeEnum = {}));
var IfcGeographicElementTypeEnum;
(function (IfcGeographicElementTypeEnum) {
    IfcGeographicElementTypeEnum["terrain"] = "terrain";
    IfcGeographicElementTypeEnum["userdefined"] = "userdefined";
    IfcGeographicElementTypeEnum["notdefined"] = "notdefined";
})(IfcGeographicElementTypeEnum || (IfcGeographicElementTypeEnum = {}));
var IfcGeometricProjectionEnum;
(function (IfcGeometricProjectionEnum) {
    IfcGeometricProjectionEnum["graph_view"] = "graph_view";
    IfcGeometricProjectionEnum["sketch_view"] = "sketch_view";
    IfcGeometricProjectionEnum["model_view"] = "model_view";
    IfcGeometricProjectionEnum["plan_view"] = "plan_view";
    IfcGeometricProjectionEnum["reflected_plan_view"] = "reflected_plan_view";
    IfcGeometricProjectionEnum["section_view"] = "section_view";
    IfcGeometricProjectionEnum["elevation_view"] = "elevation_view";
    IfcGeometricProjectionEnum["userdefined"] = "userdefined";
    IfcGeometricProjectionEnum["notdefined"] = "notdefined";
})(IfcGeometricProjectionEnum || (IfcGeometricProjectionEnum = {}));
var IfcGlobalOrLocalEnum;
(function (IfcGlobalOrLocalEnum) {
    IfcGlobalOrLocalEnum["global_coords"] = "global_coords";
    IfcGlobalOrLocalEnum["local_coords"] = "local_coords";
})(IfcGlobalOrLocalEnum || (IfcGlobalOrLocalEnum = {}));
var IfcGridTypeEnum;
(function (IfcGridTypeEnum) {
    IfcGridTypeEnum["rectangular"] = "rectangular";
    IfcGridTypeEnum["radial"] = "radial";
    IfcGridTypeEnum["triangular"] = "triangular";
    IfcGridTypeEnum["irregular"] = "irregular";
    IfcGridTypeEnum["userdefined"] = "userdefined";
    IfcGridTypeEnum["notdefined"] = "notdefined";
})(IfcGridTypeEnum || (IfcGridTypeEnum = {}));
var IfcHeatExchangerTypeEnum;
(function (IfcHeatExchangerTypeEnum) {
    IfcHeatExchangerTypeEnum["plate"] = "plate";
    IfcHeatExchangerTypeEnum["shellandtube"] = "shellandtube";
    IfcHeatExchangerTypeEnum["userdefined"] = "userdefined";
    IfcHeatExchangerTypeEnum["notdefined"] = "notdefined";
})(IfcHeatExchangerTypeEnum || (IfcHeatExchangerTypeEnum = {}));
var IfcHumidifierTypeEnum;
(function (IfcHumidifierTypeEnum) {
    IfcHumidifierTypeEnum["steaminjection"] = "steaminjection";
    IfcHumidifierTypeEnum["adiabaticairwasher"] = "adiabaticairwasher";
    IfcHumidifierTypeEnum["adiabaticpan"] = "adiabaticpan";
    IfcHumidifierTypeEnum["adiabaticwettedelement"] = "adiabaticwettedelement";
    IfcHumidifierTypeEnum["adiabaticatomizing"] = "adiabaticatomizing";
    IfcHumidifierTypeEnum["adiabaticultrasonic"] = "adiabaticultrasonic";
    IfcHumidifierTypeEnum["adiabaticrigidmedia"] = "adiabaticrigidmedia";
    IfcHumidifierTypeEnum["adiabaticcompressedairnozzle"] = "adiabaticcompressedairnozzle";
    IfcHumidifierTypeEnum["assistedelectric"] = "assistedelectric";
    IfcHumidifierTypeEnum["assistednaturalgas"] = "assistednaturalgas";
    IfcHumidifierTypeEnum["assistedpropane"] = "assistedpropane";
    IfcHumidifierTypeEnum["assistedbutane"] = "assistedbutane";
    IfcHumidifierTypeEnum["assistedsteam"] = "assistedsteam";
    IfcHumidifierTypeEnum["userdefined"] = "userdefined";
    IfcHumidifierTypeEnum["notdefined"] = "notdefined";
})(IfcHumidifierTypeEnum || (IfcHumidifierTypeEnum = {}));
var IfcInterceptorTypeEnum;
(function (IfcInterceptorTypeEnum) {
    IfcInterceptorTypeEnum["cyclonic"] = "cyclonic";
    IfcInterceptorTypeEnum["grease"] = "grease";
    IfcInterceptorTypeEnum["oil"] = "oil";
    IfcInterceptorTypeEnum["petrol"] = "petrol";
    IfcInterceptorTypeEnum["userdefined"] = "userdefined";
    IfcInterceptorTypeEnum["notdefined"] = "notdefined";
})(IfcInterceptorTypeEnum || (IfcInterceptorTypeEnum = {}));
var IfcInternalOrExternalEnum;
(function (IfcInternalOrExternalEnum) {
    IfcInternalOrExternalEnum["internal"] = "internal";
    IfcInternalOrExternalEnum["external"] = "external";
    IfcInternalOrExternalEnum["external_earth"] = "external_earth";
    IfcInternalOrExternalEnum["external_water"] = "external_water";
    IfcInternalOrExternalEnum["external_fire"] = "external_fire";
    IfcInternalOrExternalEnum["notdefined"] = "notdefined";
})(IfcInternalOrExternalEnum || (IfcInternalOrExternalEnum = {}));
var IfcInventoryTypeEnum;
(function (IfcInventoryTypeEnum) {
    IfcInventoryTypeEnum["assetinventory"] = "assetinventory";
    IfcInventoryTypeEnum["spaceinventory"] = "spaceinventory";
    IfcInventoryTypeEnum["furnitureinventory"] = "furnitureinventory";
    IfcInventoryTypeEnum["userdefined"] = "userdefined";
    IfcInventoryTypeEnum["notdefined"] = "notdefined";
})(IfcInventoryTypeEnum || (IfcInventoryTypeEnum = {}));
var IfcJunctionBoxTypeEnum;
(function (IfcJunctionBoxTypeEnum) {
    IfcJunctionBoxTypeEnum["data"] = "data";
    IfcJunctionBoxTypeEnum["power"] = "power";
    IfcJunctionBoxTypeEnum["userdefined"] = "userdefined";
    IfcJunctionBoxTypeEnum["notdefined"] = "notdefined";
})(IfcJunctionBoxTypeEnum || (IfcJunctionBoxTypeEnum = {}));
var IfcKnotType;
(function (IfcKnotType) {
    IfcKnotType["uniform_knots"] = "uniform_knots";
    IfcKnotType["quasi_uniform_knots"] = "quasi_uniform_knots";
    IfcKnotType["piecewise_bezier_knots"] = "piecewise_bezier_knots";
    IfcKnotType["unspecified"] = "unspecified";
})(IfcKnotType || (IfcKnotType = {}));
var IfcLaborResourceTypeEnum;
(function (IfcLaborResourceTypeEnum) {
    IfcLaborResourceTypeEnum["administration"] = "administration";
    IfcLaborResourceTypeEnum["carpentry"] = "carpentry";
    IfcLaborResourceTypeEnum["cleaning"] = "cleaning";
    IfcLaborResourceTypeEnum["concrete"] = "concrete";
    IfcLaborResourceTypeEnum["drywall"] = "drywall";
    IfcLaborResourceTypeEnum["electric"] = "electric";
    IfcLaborResourceTypeEnum["finishing"] = "finishing";
    IfcLaborResourceTypeEnum["flooring"] = "flooring";
    IfcLaborResourceTypeEnum["general"] = "general";
    IfcLaborResourceTypeEnum["hvac"] = "hvac";
    IfcLaborResourceTypeEnum["landscaping"] = "landscaping";
    IfcLaborResourceTypeEnum["masonry"] = "masonry";
    IfcLaborResourceTypeEnum["painting"] = "painting";
    IfcLaborResourceTypeEnum["paving"] = "paving";
    IfcLaborResourceTypeEnum["plumbing"] = "plumbing";
    IfcLaborResourceTypeEnum["roofing"] = "roofing";
    IfcLaborResourceTypeEnum["sitegrading"] = "sitegrading";
    IfcLaborResourceTypeEnum["steelwork"] = "steelwork";
    IfcLaborResourceTypeEnum["surveying"] = "surveying";
    IfcLaborResourceTypeEnum["userdefined"] = "userdefined";
    IfcLaborResourceTypeEnum["notdefined"] = "notdefined";
})(IfcLaborResourceTypeEnum || (IfcLaborResourceTypeEnum = {}));
var IfcLampTypeEnum;
(function (IfcLampTypeEnum) {
    IfcLampTypeEnum["compactfluorescent"] = "compactfluorescent";
    IfcLampTypeEnum["fluorescent"] = "fluorescent";
    IfcLampTypeEnum["halogen"] = "halogen";
    IfcLampTypeEnum["highpressuremercury"] = "highpressuremercury";
    IfcLampTypeEnum["highpressuresodium"] = "highpressuresodium";
    IfcLampTypeEnum["led"] = "led";
    IfcLampTypeEnum["metalhalide"] = "metalhalide";
    IfcLampTypeEnum["oled"] = "oled";
    IfcLampTypeEnum["tungstenfilament"] = "tungstenfilament";
    IfcLampTypeEnum["userdefined"] = "userdefined";
    IfcLampTypeEnum["notdefined"] = "notdefined";
})(IfcLampTypeEnum || (IfcLampTypeEnum = {}));
var IfcLayerSetDirectionEnum;
(function (IfcLayerSetDirectionEnum) {
    IfcLayerSetDirectionEnum["axis1"] = "axis1";
    IfcLayerSetDirectionEnum["axis2"] = "axis2";
    IfcLayerSetDirectionEnum["axis3"] = "axis3";
})(IfcLayerSetDirectionEnum || (IfcLayerSetDirectionEnum = {}));
var IfcLightDistributionCurveEnum;
(function (IfcLightDistributionCurveEnum) {
    IfcLightDistributionCurveEnum["type_a"] = "type_a";
    IfcLightDistributionCurveEnum["type_b"] = "type_b";
    IfcLightDistributionCurveEnum["type_c"] = "type_c";
    IfcLightDistributionCurveEnum["notdefined"] = "notdefined";
})(IfcLightDistributionCurveEnum || (IfcLightDistributionCurveEnum = {}));
var IfcLightEmissionSourceEnum;
(function (IfcLightEmissionSourceEnum) {
    IfcLightEmissionSourceEnum["compactfluorescent"] = "compactfluorescent";
    IfcLightEmissionSourceEnum["fluorescent"] = "fluorescent";
    IfcLightEmissionSourceEnum["highpressuremercury"] = "highpressuremercury";
    IfcLightEmissionSourceEnum["highpressuresodium"] = "highpressuresodium";
    IfcLightEmissionSourceEnum["lightemittingdiode"] = "lightemittingdiode";
    IfcLightEmissionSourceEnum["lowpressuresodium"] = "lowpressuresodium";
    IfcLightEmissionSourceEnum["lowvoltagehalogen"] = "lowvoltagehalogen";
    IfcLightEmissionSourceEnum["mainvoltagehalogen"] = "mainvoltagehalogen";
    IfcLightEmissionSourceEnum["metalhalide"] = "metalhalide";
    IfcLightEmissionSourceEnum["tungstenfilament"] = "tungstenfilament";
    IfcLightEmissionSourceEnum["notdefined"] = "notdefined";
})(IfcLightEmissionSourceEnum || (IfcLightEmissionSourceEnum = {}));
var IfcLightFixtureTypeEnum;
(function (IfcLightFixtureTypeEnum) {
    IfcLightFixtureTypeEnum["pointsource"] = "pointsource";
    IfcLightFixtureTypeEnum["directionsource"] = "directionsource";
    IfcLightFixtureTypeEnum["securitylighting"] = "securitylighting";
    IfcLightFixtureTypeEnum["userdefined"] = "userdefined";
    IfcLightFixtureTypeEnum["notdefined"] = "notdefined";
})(IfcLightFixtureTypeEnum || (IfcLightFixtureTypeEnum = {}));
var IfcLoadGroupTypeEnum;
(function (IfcLoadGroupTypeEnum) {
    IfcLoadGroupTypeEnum["load_group"] = "load_group";
    IfcLoadGroupTypeEnum["load_case"] = "load_case";
    IfcLoadGroupTypeEnum["load_combination"] = "load_combination";
    IfcLoadGroupTypeEnum["userdefined"] = "userdefined";
    IfcLoadGroupTypeEnum["notdefined"] = "notdefined";
})(IfcLoadGroupTypeEnum || (IfcLoadGroupTypeEnum = {}));
var IfcLogicalOperatorEnum;
(function (IfcLogicalOperatorEnum) {
    IfcLogicalOperatorEnum["logicaland"] = "logicaland";
    IfcLogicalOperatorEnum["logicalor"] = "logicalor";
    IfcLogicalOperatorEnum["logicalxor"] = "logicalxor";
    IfcLogicalOperatorEnum["logicalnotand"] = "logicalnotand";
    IfcLogicalOperatorEnum["logicalnotor"] = "logicalnotor";
})(IfcLogicalOperatorEnum || (IfcLogicalOperatorEnum = {}));
var IfcMechanicalFastenerTypeEnum;
(function (IfcMechanicalFastenerTypeEnum) {
    IfcMechanicalFastenerTypeEnum["anchorbolt"] = "anchorbolt";
    IfcMechanicalFastenerTypeEnum["bolt"] = "bolt";
    IfcMechanicalFastenerTypeEnum["dowel"] = "dowel";
    IfcMechanicalFastenerTypeEnum["nail"] = "nail";
    IfcMechanicalFastenerTypeEnum["nailplate"] = "nailplate";
    IfcMechanicalFastenerTypeEnum["rivet"] = "rivet";
    IfcMechanicalFastenerTypeEnum["screw"] = "screw";
    IfcMechanicalFastenerTypeEnum["shearconnector"] = "shearconnector";
    IfcMechanicalFastenerTypeEnum["staple"] = "staple";
    IfcMechanicalFastenerTypeEnum["studshearconnector"] = "studshearconnector";
    IfcMechanicalFastenerTypeEnum["userdefined"] = "userdefined";
    IfcMechanicalFastenerTypeEnum["notdefined"] = "notdefined";
})(IfcMechanicalFastenerTypeEnum || (IfcMechanicalFastenerTypeEnum = {}));
var IfcMedicalDeviceTypeEnum;
(function (IfcMedicalDeviceTypeEnum) {
    IfcMedicalDeviceTypeEnum["airstation"] = "airstation";
    IfcMedicalDeviceTypeEnum["feedairunit"] = "feedairunit";
    IfcMedicalDeviceTypeEnum["oxygengenerator"] = "oxygengenerator";
    IfcMedicalDeviceTypeEnum["oxygenplant"] = "oxygenplant";
    IfcMedicalDeviceTypeEnum["vacuumstation"] = "vacuumstation";
    IfcMedicalDeviceTypeEnum["userdefined"] = "userdefined";
    IfcMedicalDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcMedicalDeviceTypeEnum || (IfcMedicalDeviceTypeEnum = {}));
var IfcMemberTypeEnum;
(function (IfcMemberTypeEnum) {
    IfcMemberTypeEnum["brace"] = "brace";
    IfcMemberTypeEnum["chord"] = "chord";
    IfcMemberTypeEnum["collar"] = "collar";
    IfcMemberTypeEnum["member"] = "member";
    IfcMemberTypeEnum["mullion"] = "mullion";
    IfcMemberTypeEnum["plate"] = "plate";
    IfcMemberTypeEnum["post"] = "post";
    IfcMemberTypeEnum["purlin"] = "purlin";
    IfcMemberTypeEnum["rafter"] = "rafter";
    IfcMemberTypeEnum["stringer"] = "stringer";
    IfcMemberTypeEnum["strut"] = "strut";
    IfcMemberTypeEnum["stud"] = "stud";
    IfcMemberTypeEnum["userdefined"] = "userdefined";
    IfcMemberTypeEnum["notdefined"] = "notdefined";
})(IfcMemberTypeEnum || (IfcMemberTypeEnum = {}));
var IfcMotorConnectionTypeEnum;
(function (IfcMotorConnectionTypeEnum) {
    IfcMotorConnectionTypeEnum["beltdrive"] = "beltdrive";
    IfcMotorConnectionTypeEnum["coupling"] = "coupling";
    IfcMotorConnectionTypeEnum["directdrive"] = "directdrive";
    IfcMotorConnectionTypeEnum["userdefined"] = "userdefined";
    IfcMotorConnectionTypeEnum["notdefined"] = "notdefined";
})(IfcMotorConnectionTypeEnum || (IfcMotorConnectionTypeEnum = {}));
(function (IfcNullStyle) {
    IfcNullStyle["null"] = "null";
})(IfcNullStyle || (IfcNullStyle = {}));
exports.IfcNullStyle = IfcNullStyle;
var IfcObjectTypeEnum;
(function (IfcObjectTypeEnum) {
    IfcObjectTypeEnum["product"] = "product";
    IfcObjectTypeEnum["process"] = "process";
    IfcObjectTypeEnum["control"] = "control";
    IfcObjectTypeEnum["resource"] = "resource";
    IfcObjectTypeEnum["actor"] = "actor";
    IfcObjectTypeEnum["group"] = "group";
    IfcObjectTypeEnum["project"] = "project";
    IfcObjectTypeEnum["notdefined"] = "notdefined";
})(IfcObjectTypeEnum || (IfcObjectTypeEnum = {}));
var IfcObjectiveEnum;
(function (IfcObjectiveEnum) {
    IfcObjectiveEnum["codecompliance"] = "codecompliance";
    IfcObjectiveEnum["codewaiver"] = "codewaiver";
    IfcObjectiveEnum["designintent"] = "designintent";
    IfcObjectiveEnum["external"] = "external";
    IfcObjectiveEnum["healthandsafety"] = "healthandsafety";
    IfcObjectiveEnum["mergeconflict"] = "mergeconflict";
    IfcObjectiveEnum["modelview"] = "modelview";
    IfcObjectiveEnum["parameter"] = "parameter";
    IfcObjectiveEnum["requirement"] = "requirement";
    IfcObjectiveEnum["specification"] = "specification";
    IfcObjectiveEnum["triggercondition"] = "triggercondition";
    IfcObjectiveEnum["userdefined"] = "userdefined";
    IfcObjectiveEnum["notdefined"] = "notdefined";
})(IfcObjectiveEnum || (IfcObjectiveEnum = {}));
var IfcOccupantTypeEnum;
(function (IfcOccupantTypeEnum) {
    IfcOccupantTypeEnum["assignee"] = "assignee";
    IfcOccupantTypeEnum["assignor"] = "assignor";
    IfcOccupantTypeEnum["lessee"] = "lessee";
    IfcOccupantTypeEnum["lessor"] = "lessor";
    IfcOccupantTypeEnum["lettingagent"] = "lettingagent";
    IfcOccupantTypeEnum["owner"] = "owner";
    IfcOccupantTypeEnum["tenant"] = "tenant";
    IfcOccupantTypeEnum["userdefined"] = "userdefined";
    IfcOccupantTypeEnum["notdefined"] = "notdefined";
})(IfcOccupantTypeEnum || (IfcOccupantTypeEnum = {}));
var IfcOpeningElementTypeEnum;
(function (IfcOpeningElementTypeEnum) {
    IfcOpeningElementTypeEnum["opening"] = "opening";
    IfcOpeningElementTypeEnum["recess"] = "recess";
    IfcOpeningElementTypeEnum["userdefined"] = "userdefined";
    IfcOpeningElementTypeEnum["notdefined"] = "notdefined";
})(IfcOpeningElementTypeEnum || (IfcOpeningElementTypeEnum = {}));
var IfcOutletTypeEnum;
(function (IfcOutletTypeEnum) {
    IfcOutletTypeEnum["audiovisualoutlet"] = "audiovisualoutlet";
    IfcOutletTypeEnum["communicationsoutlet"] = "communicationsoutlet";
    IfcOutletTypeEnum["poweroutlet"] = "poweroutlet";
    IfcOutletTypeEnum["dataoutlet"] = "dataoutlet";
    IfcOutletTypeEnum["telephoneoutlet"] = "telephoneoutlet";
    IfcOutletTypeEnum["userdefined"] = "userdefined";
    IfcOutletTypeEnum["notdefined"] = "notdefined";
})(IfcOutletTypeEnum || (IfcOutletTypeEnum = {}));
var IfcPerformanceHistoryTypeEnum;
(function (IfcPerformanceHistoryTypeEnum) {
    IfcPerformanceHistoryTypeEnum["userdefined"] = "userdefined";
    IfcPerformanceHistoryTypeEnum["notdefined"] = "notdefined";
})(IfcPerformanceHistoryTypeEnum || (IfcPerformanceHistoryTypeEnum = {}));
var IfcPermeableCoveringOperationEnum;
(function (IfcPermeableCoveringOperationEnum) {
    IfcPermeableCoveringOperationEnum["grill"] = "grill";
    IfcPermeableCoveringOperationEnum["louver"] = "louver";
    IfcPermeableCoveringOperationEnum["screen"] = "screen";
    IfcPermeableCoveringOperationEnum["userdefined"] = "userdefined";
    IfcPermeableCoveringOperationEnum["notdefined"] = "notdefined";
})(IfcPermeableCoveringOperationEnum || (IfcPermeableCoveringOperationEnum = {}));
var IfcPermitTypeEnum;
(function (IfcPermitTypeEnum) {
    IfcPermitTypeEnum["access"] = "access";
    IfcPermitTypeEnum["building"] = "building";
    IfcPermitTypeEnum["work"] = "work";
    IfcPermitTypeEnum["userdefined"] = "userdefined";
    IfcPermitTypeEnum["notdefined"] = "notdefined";
})(IfcPermitTypeEnum || (IfcPermitTypeEnum = {}));
var IfcPhysicalOrVirtualEnum;
(function (IfcPhysicalOrVirtualEnum) {
    IfcPhysicalOrVirtualEnum["physical"] = "physical";
    IfcPhysicalOrVirtualEnum["virtual"] = "virtual";
    IfcPhysicalOrVirtualEnum["notdefined"] = "notdefined";
})(IfcPhysicalOrVirtualEnum || (IfcPhysicalOrVirtualEnum = {}));
var IfcPileConstructionEnum;
(function (IfcPileConstructionEnum) {
    IfcPileConstructionEnum["cast_in_place"] = "cast_in_place";
    IfcPileConstructionEnum["composite"] = "composite";
    IfcPileConstructionEnum["precast_concrete"] = "precast_concrete";
    IfcPileConstructionEnum["prefab_steel"] = "prefab_steel";
    IfcPileConstructionEnum["userdefined"] = "userdefined";
    IfcPileConstructionEnum["notdefined"] = "notdefined";
})(IfcPileConstructionEnum || (IfcPileConstructionEnum = {}));
var IfcPileTypeEnum;
(function (IfcPileTypeEnum) {
    IfcPileTypeEnum["bored"] = "bored";
    IfcPileTypeEnum["driven"] = "driven";
    IfcPileTypeEnum["jetgrouting"] = "jetgrouting";
    IfcPileTypeEnum["cohesion"] = "cohesion";
    IfcPileTypeEnum["friction"] = "friction";
    IfcPileTypeEnum["support"] = "support";
    IfcPileTypeEnum["userdefined"] = "userdefined";
    IfcPileTypeEnum["notdefined"] = "notdefined";
})(IfcPileTypeEnum || (IfcPileTypeEnum = {}));
var IfcPipeFittingTypeEnum;
(function (IfcPipeFittingTypeEnum) {
    IfcPipeFittingTypeEnum["bend"] = "bend";
    IfcPipeFittingTypeEnum["connector"] = "connector";
    IfcPipeFittingTypeEnum["entry"] = "entry";
    IfcPipeFittingTypeEnum["exit"] = "exit";
    IfcPipeFittingTypeEnum["junction"] = "junction";
    IfcPipeFittingTypeEnum["obstruction"] = "obstruction";
    IfcPipeFittingTypeEnum["transition"] = "transition";
    IfcPipeFittingTypeEnum["userdefined"] = "userdefined";
    IfcPipeFittingTypeEnum["notdefined"] = "notdefined";
})(IfcPipeFittingTypeEnum || (IfcPipeFittingTypeEnum = {}));
var IfcPipeSegmentTypeEnum;
(function (IfcPipeSegmentTypeEnum) {
    IfcPipeSegmentTypeEnum["culvert"] = "culvert";
    IfcPipeSegmentTypeEnum["flexiblesegment"] = "flexiblesegment";
    IfcPipeSegmentTypeEnum["rigidsegment"] = "rigidsegment";
    IfcPipeSegmentTypeEnum["gutter"] = "gutter";
    IfcPipeSegmentTypeEnum["spool"] = "spool";
    IfcPipeSegmentTypeEnum["userdefined"] = "userdefined";
    IfcPipeSegmentTypeEnum["notdefined"] = "notdefined";
})(IfcPipeSegmentTypeEnum || (IfcPipeSegmentTypeEnum = {}));
var IfcPlateTypeEnum;
(function (IfcPlateTypeEnum) {
    IfcPlateTypeEnum["curtain_panel"] = "curtain_panel";
    IfcPlateTypeEnum["sheet"] = "sheet";
    IfcPlateTypeEnum["userdefined"] = "userdefined";
    IfcPlateTypeEnum["notdefined"] = "notdefined";
})(IfcPlateTypeEnum || (IfcPlateTypeEnum = {}));
var IfcPreferredSurfaceCurveRepresentation;
(function (IfcPreferredSurfaceCurveRepresentation) {
    IfcPreferredSurfaceCurveRepresentation["curve3d"] = "curve3d";
    IfcPreferredSurfaceCurveRepresentation["pcurve_s1"] = "pcurve_s1";
    IfcPreferredSurfaceCurveRepresentation["pcurve_s2"] = "pcurve_s2";
})(IfcPreferredSurfaceCurveRepresentation || (IfcPreferredSurfaceCurveRepresentation = {}));
var IfcProcedureTypeEnum;
(function (IfcProcedureTypeEnum) {
    IfcProcedureTypeEnum["advice_caution"] = "advice_caution";
    IfcProcedureTypeEnum["advice_note"] = "advice_note";
    IfcProcedureTypeEnum["advice_warning"] = "advice_warning";
    IfcProcedureTypeEnum["calibration"] = "calibration";
    IfcProcedureTypeEnum["diagnostic"] = "diagnostic";
    IfcProcedureTypeEnum["shutdown"] = "shutdown";
    IfcProcedureTypeEnum["startup"] = "startup";
    IfcProcedureTypeEnum["userdefined"] = "userdefined";
    IfcProcedureTypeEnum["notdefined"] = "notdefined";
})(IfcProcedureTypeEnum || (IfcProcedureTypeEnum = {}));
var IfcProfileTypeEnum;
(function (IfcProfileTypeEnum) {
    IfcProfileTypeEnum["curve"] = "curve";
    IfcProfileTypeEnum["area"] = "area";
})(IfcProfileTypeEnum || (IfcProfileTypeEnum = {}));
var IfcProjectOrderTypeEnum;
(function (IfcProjectOrderTypeEnum) {
    IfcProjectOrderTypeEnum["changeorder"] = "changeorder";
    IfcProjectOrderTypeEnum["maintenanceworkorder"] = "maintenanceworkorder";
    IfcProjectOrderTypeEnum["moveorder"] = "moveorder";
    IfcProjectOrderTypeEnum["purchaseorder"] = "purchaseorder";
    IfcProjectOrderTypeEnum["workorder"] = "workorder";
    IfcProjectOrderTypeEnum["userdefined"] = "userdefined";
    IfcProjectOrderTypeEnum["notdefined"] = "notdefined";
})(IfcProjectOrderTypeEnum || (IfcProjectOrderTypeEnum = {}));
var IfcProjectedOrTrueLengthEnum;
(function (IfcProjectedOrTrueLengthEnum) {
    IfcProjectedOrTrueLengthEnum["projected_length"] = "projected_length";
    IfcProjectedOrTrueLengthEnum["true_length"] = "true_length";
})(IfcProjectedOrTrueLengthEnum || (IfcProjectedOrTrueLengthEnum = {}));
var IfcProjectionElementTypeEnum;
(function (IfcProjectionElementTypeEnum) {
    IfcProjectionElementTypeEnum["userdefined"] = "userdefined";
    IfcProjectionElementTypeEnum["notdefined"] = "notdefined";
})(IfcProjectionElementTypeEnum || (IfcProjectionElementTypeEnum = {}));
var IfcPropertySetTemplateTypeEnum;
(function (IfcPropertySetTemplateTypeEnum) {
    IfcPropertySetTemplateTypeEnum["pset_typedrivenonly"] = "pset_typedrivenonly";
    IfcPropertySetTemplateTypeEnum["pset_typedrivenoverride"] = "pset_typedrivenoverride";
    IfcPropertySetTemplateTypeEnum["pset_occurrencedriven"] = "pset_occurrencedriven";
    IfcPropertySetTemplateTypeEnum["pset_performancedriven"] = "pset_performancedriven";
    IfcPropertySetTemplateTypeEnum["qto_typedrivenonly"] = "qto_typedrivenonly";
    IfcPropertySetTemplateTypeEnum["qto_typedrivenoverride"] = "qto_typedrivenoverride";
    IfcPropertySetTemplateTypeEnum["qto_occurrencedriven"] = "qto_occurrencedriven";
    IfcPropertySetTemplateTypeEnum["notdefined"] = "notdefined";
})(IfcPropertySetTemplateTypeEnum || (IfcPropertySetTemplateTypeEnum = {}));
var IfcProtectiveDeviceTrippingUnitTypeEnum;
(function (IfcProtectiveDeviceTrippingUnitTypeEnum) {
    IfcProtectiveDeviceTrippingUnitTypeEnum["electronic"] = "electronic";
    IfcProtectiveDeviceTrippingUnitTypeEnum["electromagnetic"] = "electromagnetic";
    IfcProtectiveDeviceTrippingUnitTypeEnum["residualcurrent"] = "residualcurrent";
    IfcProtectiveDeviceTrippingUnitTypeEnum["thermal"] = "thermal";
    IfcProtectiveDeviceTrippingUnitTypeEnum["userdefined"] = "userdefined";
    IfcProtectiveDeviceTrippingUnitTypeEnum["notdefined"] = "notdefined";
})(IfcProtectiveDeviceTrippingUnitTypeEnum || (IfcProtectiveDeviceTrippingUnitTypeEnum = {}));
var IfcProtectiveDeviceTypeEnum;
(function (IfcProtectiveDeviceTypeEnum) {
    IfcProtectiveDeviceTypeEnum["circuitbreaker"] = "circuitbreaker";
    IfcProtectiveDeviceTypeEnum["earthleakagecircuitbreaker"] = "earthleakagecircuitbreaker";
    IfcProtectiveDeviceTypeEnum["earthingswitch"] = "earthingswitch";
    IfcProtectiveDeviceTypeEnum["fusedisconnector"] = "fusedisconnector";
    IfcProtectiveDeviceTypeEnum["residualcurrentcircuitbreaker"] = "residualcurrentcircuitbreaker";
    IfcProtectiveDeviceTypeEnum["residualcurrentswitch"] = "residualcurrentswitch";
    IfcProtectiveDeviceTypeEnum["varistor"] = "varistor";
    IfcProtectiveDeviceTypeEnum["userdefined"] = "userdefined";
    IfcProtectiveDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcProtectiveDeviceTypeEnum || (IfcProtectiveDeviceTypeEnum = {}));
var IfcPumpTypeEnum;
(function (IfcPumpTypeEnum) {
    IfcPumpTypeEnum["circulator"] = "circulator";
    IfcPumpTypeEnum["endsuction"] = "endsuction";
    IfcPumpTypeEnum["splitcase"] = "splitcase";
    IfcPumpTypeEnum["submersiblepump"] = "submersiblepump";
    IfcPumpTypeEnum["sumppump"] = "sumppump";
    IfcPumpTypeEnum["verticalinline"] = "verticalinline";
    IfcPumpTypeEnum["verticalturbine"] = "verticalturbine";
    IfcPumpTypeEnum["userdefined"] = "userdefined";
    IfcPumpTypeEnum["notdefined"] = "notdefined";
})(IfcPumpTypeEnum || (IfcPumpTypeEnum = {}));
var IfcRailingTypeEnum;
(function (IfcRailingTypeEnum) {
    IfcRailingTypeEnum["handrail"] = "handrail";
    IfcRailingTypeEnum["guardrail"] = "guardrail";
    IfcRailingTypeEnum["balustrade"] = "balustrade";
    IfcRailingTypeEnum["userdefined"] = "userdefined";
    IfcRailingTypeEnum["notdefined"] = "notdefined";
})(IfcRailingTypeEnum || (IfcRailingTypeEnum = {}));
var IfcRampFlightTypeEnum;
(function (IfcRampFlightTypeEnum) {
    IfcRampFlightTypeEnum["straight"] = "straight";
    IfcRampFlightTypeEnum["spiral"] = "spiral";
    IfcRampFlightTypeEnum["userdefined"] = "userdefined";
    IfcRampFlightTypeEnum["notdefined"] = "notdefined";
})(IfcRampFlightTypeEnum || (IfcRampFlightTypeEnum = {}));
var IfcRampTypeEnum;
(function (IfcRampTypeEnum) {
    IfcRampTypeEnum["straight_run_ramp"] = "straight_run_ramp";
    IfcRampTypeEnum["two_straight_run_ramp"] = "two_straight_run_ramp";
    IfcRampTypeEnum["quarter_turn_ramp"] = "quarter_turn_ramp";
    IfcRampTypeEnum["two_quarter_turn_ramp"] = "two_quarter_turn_ramp";
    IfcRampTypeEnum["half_turn_ramp"] = "half_turn_ramp";
    IfcRampTypeEnum["spiral_ramp"] = "spiral_ramp";
    IfcRampTypeEnum["userdefined"] = "userdefined";
    IfcRampTypeEnum["notdefined"] = "notdefined";
})(IfcRampTypeEnum || (IfcRampTypeEnum = {}));
var IfcRecurrenceTypeEnum;
(function (IfcRecurrenceTypeEnum) {
    IfcRecurrenceTypeEnum["daily"] = "daily";
    IfcRecurrenceTypeEnum["weekly"] = "weekly";
    IfcRecurrenceTypeEnum["monthly_by_day_of_month"] = "monthly_by_day_of_month";
    IfcRecurrenceTypeEnum["monthly_by_position"] = "monthly_by_position";
    IfcRecurrenceTypeEnum["by_day_count"] = "by_day_count";
    IfcRecurrenceTypeEnum["by_weekday_count"] = "by_weekday_count";
    IfcRecurrenceTypeEnum["yearly_by_day_of_month"] = "yearly_by_day_of_month";
    IfcRecurrenceTypeEnum["yearly_by_position"] = "yearly_by_position";
})(IfcRecurrenceTypeEnum || (IfcRecurrenceTypeEnum = {}));
var IfcReflectanceMethodEnum;
(function (IfcReflectanceMethodEnum) {
    IfcReflectanceMethodEnum["blinn"] = "blinn";
    IfcReflectanceMethodEnum["flat"] = "flat";
    IfcReflectanceMethodEnum["glass"] = "glass";
    IfcReflectanceMethodEnum["matt"] = "matt";
    IfcReflectanceMethodEnum["metal"] = "metal";
    IfcReflectanceMethodEnum["mirror"] = "mirror";
    IfcReflectanceMethodEnum["phong"] = "phong";
    IfcReflectanceMethodEnum["plastic"] = "plastic";
    IfcReflectanceMethodEnum["strauss"] = "strauss";
    IfcReflectanceMethodEnum["notdefined"] = "notdefined";
})(IfcReflectanceMethodEnum || (IfcReflectanceMethodEnum = {}));
var IfcReinforcingBarRoleEnum;
(function (IfcReinforcingBarRoleEnum) {
    IfcReinforcingBarRoleEnum["main"] = "main";
    IfcReinforcingBarRoleEnum["shear"] = "shear";
    IfcReinforcingBarRoleEnum["ligature"] = "ligature";
    IfcReinforcingBarRoleEnum["stud"] = "stud";
    IfcReinforcingBarRoleEnum["punching"] = "punching";
    IfcReinforcingBarRoleEnum["edge"] = "edge";
    IfcReinforcingBarRoleEnum["ring"] = "ring";
    IfcReinforcingBarRoleEnum["anchoring"] = "anchoring";
    IfcReinforcingBarRoleEnum["userdefined"] = "userdefined";
    IfcReinforcingBarRoleEnum["notdefined"] = "notdefined";
})(IfcReinforcingBarRoleEnum || (IfcReinforcingBarRoleEnum = {}));
var IfcReinforcingBarSurfaceEnum;
(function (IfcReinforcingBarSurfaceEnum) {
    IfcReinforcingBarSurfaceEnum["plain"] = "plain";
    IfcReinforcingBarSurfaceEnum["textured"] = "textured";
})(IfcReinforcingBarSurfaceEnum || (IfcReinforcingBarSurfaceEnum = {}));
var IfcReinforcingBarTypeEnum;
(function (IfcReinforcingBarTypeEnum) {
    IfcReinforcingBarTypeEnum["anchoring"] = "anchoring";
    IfcReinforcingBarTypeEnum["edge"] = "edge";
    IfcReinforcingBarTypeEnum["ligature"] = "ligature";
    IfcReinforcingBarTypeEnum["main"] = "main";
    IfcReinforcingBarTypeEnum["punching"] = "punching";
    IfcReinforcingBarTypeEnum["ring"] = "ring";
    IfcReinforcingBarTypeEnum["shear"] = "shear";
    IfcReinforcingBarTypeEnum["stud"] = "stud";
    IfcReinforcingBarTypeEnum["userdefined"] = "userdefined";
    IfcReinforcingBarTypeEnum["notdefined"] = "notdefined";
})(IfcReinforcingBarTypeEnum || (IfcReinforcingBarTypeEnum = {}));
var IfcReinforcingMeshTypeEnum;
(function (IfcReinforcingMeshTypeEnum) {
    IfcReinforcingMeshTypeEnum["userdefined"] = "userdefined";
    IfcReinforcingMeshTypeEnum["notdefined"] = "notdefined";
})(IfcReinforcingMeshTypeEnum || (IfcReinforcingMeshTypeEnum = {}));
var IfcRoleEnum;
(function (IfcRoleEnum) {
    IfcRoleEnum["supplier"] = "supplier";
    IfcRoleEnum["manufacturer"] = "manufacturer";
    IfcRoleEnum["contractor"] = "contractor";
    IfcRoleEnum["subcontractor"] = "subcontractor";
    IfcRoleEnum["architect"] = "architect";
    IfcRoleEnum["structuralengineer"] = "structuralengineer";
    IfcRoleEnum["costengineer"] = "costengineer";
    IfcRoleEnum["client"] = "client";
    IfcRoleEnum["buildingowner"] = "buildingowner";
    IfcRoleEnum["buildingoperator"] = "buildingoperator";
    IfcRoleEnum["mechanicalengineer"] = "mechanicalengineer";
    IfcRoleEnum["electricalengineer"] = "electricalengineer";
    IfcRoleEnum["projectmanager"] = "projectmanager";
    IfcRoleEnum["facilitiesmanager"] = "facilitiesmanager";
    IfcRoleEnum["civilengineer"] = "civilengineer";
    IfcRoleEnum["commissioningengineer"] = "commissioningengineer";
    IfcRoleEnum["engineer"] = "engineer";
    IfcRoleEnum["owner"] = "owner";
    IfcRoleEnum["consultant"] = "consultant";
    IfcRoleEnum["constructionmanager"] = "constructionmanager";
    IfcRoleEnum["fieldconstructionmanager"] = "fieldconstructionmanager";
    IfcRoleEnum["reseller"] = "reseller";
    IfcRoleEnum["userdefined"] = "userdefined";
})(IfcRoleEnum || (IfcRoleEnum = {}));
var IfcRoofTypeEnum;
(function (IfcRoofTypeEnum) {
    IfcRoofTypeEnum["flat_roof"] = "flat_roof";
    IfcRoofTypeEnum["shed_roof"] = "shed_roof";
    IfcRoofTypeEnum["gable_roof"] = "gable_roof";
    IfcRoofTypeEnum["hip_roof"] = "hip_roof";
    IfcRoofTypeEnum["hipped_gable_roof"] = "hipped_gable_roof";
    IfcRoofTypeEnum["gambrel_roof"] = "gambrel_roof";
    IfcRoofTypeEnum["mansard_roof"] = "mansard_roof";
    IfcRoofTypeEnum["barrel_roof"] = "barrel_roof";
    IfcRoofTypeEnum["rainbow_roof"] = "rainbow_roof";
    IfcRoofTypeEnum["butterfly_roof"] = "butterfly_roof";
    IfcRoofTypeEnum["pavilion_roof"] = "pavilion_roof";
    IfcRoofTypeEnum["dome_roof"] = "dome_roof";
    IfcRoofTypeEnum["freeform"] = "freeform";
    IfcRoofTypeEnum["userdefined"] = "userdefined";
    IfcRoofTypeEnum["notdefined"] = "notdefined";
})(IfcRoofTypeEnum || (IfcRoofTypeEnum = {}));
var IfcSIPrefix;
(function (IfcSIPrefix) {
    IfcSIPrefix["exa"] = "exa";
    IfcSIPrefix["peta"] = "peta";
    IfcSIPrefix["tera"] = "tera";
    IfcSIPrefix["giga"] = "giga";
    IfcSIPrefix["mega"] = "mega";
    IfcSIPrefix["kilo"] = "kilo";
    IfcSIPrefix["hecto"] = "hecto";
    IfcSIPrefix["deca"] = "deca";
    IfcSIPrefix["deci"] = "deci";
    IfcSIPrefix["centi"] = "centi";
    IfcSIPrefix["milli"] = "milli";
    IfcSIPrefix["micro"] = "micro";
    IfcSIPrefix["nano"] = "nano";
    IfcSIPrefix["pico"] = "pico";
    IfcSIPrefix["femto"] = "femto";
    IfcSIPrefix["atto"] = "atto";
})(IfcSIPrefix || (IfcSIPrefix = {}));
var IfcSIUnitName;
(function (IfcSIUnitName) {
    IfcSIUnitName["ampere"] = "ampere";
    IfcSIUnitName["becquerel"] = "becquerel";
    IfcSIUnitName["candela"] = "candela";
    IfcSIUnitName["coulomb"] = "coulomb";
    IfcSIUnitName["cubic_metre"] = "cubic_metre";
    IfcSIUnitName["degree_celsius"] = "degree_celsius";
    IfcSIUnitName["farad"] = "farad";
    IfcSIUnitName["gram"] = "gram";
    IfcSIUnitName["gray"] = "gray";
    IfcSIUnitName["henry"] = "henry";
    IfcSIUnitName["hertz"] = "hertz";
    IfcSIUnitName["joule"] = "joule";
    IfcSIUnitName["kelvin"] = "kelvin";
    IfcSIUnitName["lumen"] = "lumen";
    IfcSIUnitName["lux"] = "lux";
    IfcSIUnitName["metre"] = "metre";
    IfcSIUnitName["mole"] = "mole";
    IfcSIUnitName["newton"] = "newton";
    IfcSIUnitName["ohm"] = "ohm";
    IfcSIUnitName["pascal"] = "pascal";
    IfcSIUnitName["radian"] = "radian";
    IfcSIUnitName["second"] = "second";
    IfcSIUnitName["siemens"] = "siemens";
    IfcSIUnitName["sievert"] = "sievert";
    IfcSIUnitName["square_metre"] = "square_metre";
    IfcSIUnitName["steradian"] = "steradian";
    IfcSIUnitName["tesla"] = "tesla";
    IfcSIUnitName["volt"] = "volt";
    IfcSIUnitName["watt"] = "watt";
    IfcSIUnitName["weber"] = "weber";
})(IfcSIUnitName || (IfcSIUnitName = {}));
var IfcSanitaryTerminalTypeEnum;
(function (IfcSanitaryTerminalTypeEnum) {
    IfcSanitaryTerminalTypeEnum["bath"] = "bath";
    IfcSanitaryTerminalTypeEnum["bidet"] = "bidet";
    IfcSanitaryTerminalTypeEnum["cistern"] = "cistern";
    IfcSanitaryTerminalTypeEnum["shower"] = "shower";
    IfcSanitaryTerminalTypeEnum["sink"] = "sink";
    IfcSanitaryTerminalTypeEnum["sanitaryfountain"] = "sanitaryfountain";
    IfcSanitaryTerminalTypeEnum["toiletpan"] = "toiletpan";
    IfcSanitaryTerminalTypeEnum["urinal"] = "urinal";
    IfcSanitaryTerminalTypeEnum["washhandbasin"] = "washhandbasin";
    IfcSanitaryTerminalTypeEnum["wcseat"] = "wcseat";
    IfcSanitaryTerminalTypeEnum["userdefined"] = "userdefined";
    IfcSanitaryTerminalTypeEnum["notdefined"] = "notdefined";
})(IfcSanitaryTerminalTypeEnum || (IfcSanitaryTerminalTypeEnum = {}));
var IfcSectionTypeEnum;
(function (IfcSectionTypeEnum) {
    IfcSectionTypeEnum["uniform"] = "uniform";
    IfcSectionTypeEnum["tapered"] = "tapered";
})(IfcSectionTypeEnum || (IfcSectionTypeEnum = {}));
var IfcSensorTypeEnum;
(function (IfcSensorTypeEnum) {
    IfcSensorTypeEnum["cosensor"] = "cosensor";
    IfcSensorTypeEnum["co2sensor"] = "co2sensor";
    IfcSensorTypeEnum["conductancesensor"] = "conductancesensor";
    IfcSensorTypeEnum["contactsensor"] = "contactsensor";
    IfcSensorTypeEnum["firesensor"] = "firesensor";
    IfcSensorTypeEnum["flowsensor"] = "flowsensor";
    IfcSensorTypeEnum["frostsensor"] = "frostsensor";
    IfcSensorTypeEnum["gassensor"] = "gassensor";
    IfcSensorTypeEnum["heatsensor"] = "heatsensor";
    IfcSensorTypeEnum["humiditysensor"] = "humiditysensor";
    IfcSensorTypeEnum["identifiersensor"] = "identifiersensor";
    IfcSensorTypeEnum["ionconcentrationsensor"] = "ionconcentrationsensor";
    IfcSensorTypeEnum["levelsensor"] = "levelsensor";
    IfcSensorTypeEnum["lightsensor"] = "lightsensor";
    IfcSensorTypeEnum["moisturesensor"] = "moisturesensor";
    IfcSensorTypeEnum["movementsensor"] = "movementsensor";
    IfcSensorTypeEnum["phsensor"] = "phsensor";
    IfcSensorTypeEnum["pressuresensor"] = "pressuresensor";
    IfcSensorTypeEnum["radiationsensor"] = "radiationsensor";
    IfcSensorTypeEnum["radioactivitysensor"] = "radioactivitysensor";
    IfcSensorTypeEnum["smokesensor"] = "smokesensor";
    IfcSensorTypeEnum["soundsensor"] = "soundsensor";
    IfcSensorTypeEnum["temperaturesensor"] = "temperaturesensor";
    IfcSensorTypeEnum["windsensor"] = "windsensor";
    IfcSensorTypeEnum["userdefined"] = "userdefined";
    IfcSensorTypeEnum["notdefined"] = "notdefined";
})(IfcSensorTypeEnum || (IfcSensorTypeEnum = {}));
var IfcSequenceEnum;
(function (IfcSequenceEnum) {
    IfcSequenceEnum["start_start"] = "start_start";
    IfcSequenceEnum["start_finish"] = "start_finish";
    IfcSequenceEnum["finish_start"] = "finish_start";
    IfcSequenceEnum["finish_finish"] = "finish_finish";
    IfcSequenceEnum["userdefined"] = "userdefined";
    IfcSequenceEnum["notdefined"] = "notdefined";
})(IfcSequenceEnum || (IfcSequenceEnum = {}));
var IfcShadingDeviceTypeEnum;
(function (IfcShadingDeviceTypeEnum) {
    IfcShadingDeviceTypeEnum["jalousie"] = "jalousie";
    IfcShadingDeviceTypeEnum["shutter"] = "shutter";
    IfcShadingDeviceTypeEnum["awning"] = "awning";
    IfcShadingDeviceTypeEnum["userdefined"] = "userdefined";
    IfcShadingDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcShadingDeviceTypeEnum || (IfcShadingDeviceTypeEnum = {}));
var IfcSimplePropertyTemplateTypeEnum;
(function (IfcSimplePropertyTemplateTypeEnum) {
    IfcSimplePropertyTemplateTypeEnum["p_singlevalue"] = "p_singlevalue";
    IfcSimplePropertyTemplateTypeEnum["p_enumeratedvalue"] = "p_enumeratedvalue";
    IfcSimplePropertyTemplateTypeEnum["p_boundedvalue"] = "p_boundedvalue";
    IfcSimplePropertyTemplateTypeEnum["p_listvalue"] = "p_listvalue";
    IfcSimplePropertyTemplateTypeEnum["p_tablevalue"] = "p_tablevalue";
    IfcSimplePropertyTemplateTypeEnum["p_referencevalue"] = "p_referencevalue";
    IfcSimplePropertyTemplateTypeEnum["q_length"] = "q_length";
    IfcSimplePropertyTemplateTypeEnum["q_area"] = "q_area";
    IfcSimplePropertyTemplateTypeEnum["q_volume"] = "q_volume";
    IfcSimplePropertyTemplateTypeEnum["q_count"] = "q_count";
    IfcSimplePropertyTemplateTypeEnum["q_weight"] = "q_weight";
    IfcSimplePropertyTemplateTypeEnum["q_time"] = "q_time";
})(IfcSimplePropertyTemplateTypeEnum || (IfcSimplePropertyTemplateTypeEnum = {}));
var IfcSlabTypeEnum;
(function (IfcSlabTypeEnum) {
    IfcSlabTypeEnum["floor"] = "floor";
    IfcSlabTypeEnum["roof"] = "roof";
    IfcSlabTypeEnum["landing"] = "landing";
    IfcSlabTypeEnum["baseslab"] = "baseslab";
    IfcSlabTypeEnum["userdefined"] = "userdefined";
    IfcSlabTypeEnum["notdefined"] = "notdefined";
})(IfcSlabTypeEnum || (IfcSlabTypeEnum = {}));
var IfcSolarDeviceTypeEnum;
(function (IfcSolarDeviceTypeEnum) {
    IfcSolarDeviceTypeEnum["solarcollector"] = "solarcollector";
    IfcSolarDeviceTypeEnum["solarpanel"] = "solarpanel";
    IfcSolarDeviceTypeEnum["userdefined"] = "userdefined";
    IfcSolarDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcSolarDeviceTypeEnum || (IfcSolarDeviceTypeEnum = {}));
var IfcSpaceHeaterTypeEnum;
(function (IfcSpaceHeaterTypeEnum) {
    IfcSpaceHeaterTypeEnum["convector"] = "convector";
    IfcSpaceHeaterTypeEnum["radiator"] = "radiator";
    IfcSpaceHeaterTypeEnum["userdefined"] = "userdefined";
    IfcSpaceHeaterTypeEnum["notdefined"] = "notdefined";
})(IfcSpaceHeaterTypeEnum || (IfcSpaceHeaterTypeEnum = {}));
var IfcSpaceTypeEnum;
(function (IfcSpaceTypeEnum) {
    IfcSpaceTypeEnum["space"] = "space";
    IfcSpaceTypeEnum["parking"] = "parking";
    IfcSpaceTypeEnum["gfa"] = "gfa";
    IfcSpaceTypeEnum["internal"] = "internal";
    IfcSpaceTypeEnum["external"] = "external";
    IfcSpaceTypeEnum["userdefined"] = "userdefined";
    IfcSpaceTypeEnum["notdefined"] = "notdefined";
})(IfcSpaceTypeEnum || (IfcSpaceTypeEnum = {}));
var IfcSpatialZoneTypeEnum;
(function (IfcSpatialZoneTypeEnum) {
    IfcSpatialZoneTypeEnum["construction"] = "construction";
    IfcSpatialZoneTypeEnum["firesafety"] = "firesafety";
    IfcSpatialZoneTypeEnum["lighting"] = "lighting";
    IfcSpatialZoneTypeEnum["occupancy"] = "occupancy";
    IfcSpatialZoneTypeEnum["security"] = "security";
    IfcSpatialZoneTypeEnum["thermal"] = "thermal";
    IfcSpatialZoneTypeEnum["transport"] = "transport";
    IfcSpatialZoneTypeEnum["ventilation"] = "ventilation";
    IfcSpatialZoneTypeEnum["userdefined"] = "userdefined";
    IfcSpatialZoneTypeEnum["notdefined"] = "notdefined";
})(IfcSpatialZoneTypeEnum || (IfcSpatialZoneTypeEnum = {}));
var IfcStackTerminalTypeEnum;
(function (IfcStackTerminalTypeEnum) {
    IfcStackTerminalTypeEnum["birdcage"] = "birdcage";
    IfcStackTerminalTypeEnum["cowl"] = "cowl";
    IfcStackTerminalTypeEnum["rainwaterhopper"] = "rainwaterhopper";
    IfcStackTerminalTypeEnum["userdefined"] = "userdefined";
    IfcStackTerminalTypeEnum["notdefined"] = "notdefined";
})(IfcStackTerminalTypeEnum || (IfcStackTerminalTypeEnum = {}));
var IfcStairFlightTypeEnum;
(function (IfcStairFlightTypeEnum) {
    IfcStairFlightTypeEnum["straight"] = "straight";
    IfcStairFlightTypeEnum["winder"] = "winder";
    IfcStairFlightTypeEnum["spiral"] = "spiral";
    IfcStairFlightTypeEnum["curved"] = "curved";
    IfcStairFlightTypeEnum["freeform"] = "freeform";
    IfcStairFlightTypeEnum["userdefined"] = "userdefined";
    IfcStairFlightTypeEnum["notdefined"] = "notdefined";
})(IfcStairFlightTypeEnum || (IfcStairFlightTypeEnum = {}));
var IfcStairTypeEnum;
(function (IfcStairTypeEnum) {
    IfcStairTypeEnum["straight_run_stair"] = "straight_run_stair";
    IfcStairTypeEnum["two_straight_run_stair"] = "two_straight_run_stair";
    IfcStairTypeEnum["quarter_winding_stair"] = "quarter_winding_stair";
    IfcStairTypeEnum["quarter_turn_stair"] = "quarter_turn_stair";
    IfcStairTypeEnum["half_winding_stair"] = "half_winding_stair";
    IfcStairTypeEnum["half_turn_stair"] = "half_turn_stair";
    IfcStairTypeEnum["two_quarter_winding_stair"] = "two_quarter_winding_stair";
    IfcStairTypeEnum["two_quarter_turn_stair"] = "two_quarter_turn_stair";
    IfcStairTypeEnum["three_quarter_winding_stair"] = "three_quarter_winding_stair";
    IfcStairTypeEnum["three_quarter_turn_stair"] = "three_quarter_turn_stair";
    IfcStairTypeEnum["spiral_stair"] = "spiral_stair";
    IfcStairTypeEnum["double_return_stair"] = "double_return_stair";
    IfcStairTypeEnum["curved_run_stair"] = "curved_run_stair";
    IfcStairTypeEnum["two_curved_run_stair"] = "two_curved_run_stair";
    IfcStairTypeEnum["userdefined"] = "userdefined";
    IfcStairTypeEnum["notdefined"] = "notdefined";
})(IfcStairTypeEnum || (IfcStairTypeEnum = {}));
var IfcStateEnum;
(function (IfcStateEnum) {
    IfcStateEnum["readwrite"] = "readwrite";
    IfcStateEnum["readonly"] = "readonly";
    IfcStateEnum["locked"] = "locked";
    IfcStateEnum["readwritelocked"] = "readwritelocked";
    IfcStateEnum["readonlylocked"] = "readonlylocked";
})(IfcStateEnum || (IfcStateEnum = {}));
var IfcStructuralCurveActivityTypeEnum;
(function (IfcStructuralCurveActivityTypeEnum) {
    IfcStructuralCurveActivityTypeEnum["const"] = "const";
    IfcStructuralCurveActivityTypeEnum["linear"] = "linear";
    IfcStructuralCurveActivityTypeEnum["polygonal"] = "polygonal";
    IfcStructuralCurveActivityTypeEnum["equidistant"] = "equidistant";
    IfcStructuralCurveActivityTypeEnum["sinus"] = "sinus";
    IfcStructuralCurveActivityTypeEnum["parabola"] = "parabola";
    IfcStructuralCurveActivityTypeEnum["discrete"] = "discrete";
    IfcStructuralCurveActivityTypeEnum["userdefined"] = "userdefined";
    IfcStructuralCurveActivityTypeEnum["notdefined"] = "notdefined";
})(IfcStructuralCurveActivityTypeEnum || (IfcStructuralCurveActivityTypeEnum = {}));
var IfcStructuralCurveMemberTypeEnum;
(function (IfcStructuralCurveMemberTypeEnum) {
    IfcStructuralCurveMemberTypeEnum["rigid_joined_member"] = "rigid_joined_member";
    IfcStructuralCurveMemberTypeEnum["pin_joined_member"] = "pin_joined_member";
    IfcStructuralCurveMemberTypeEnum["cable"] = "cable";
    IfcStructuralCurveMemberTypeEnum["tension_member"] = "tension_member";
    IfcStructuralCurveMemberTypeEnum["compression_member"] = "compression_member";
    IfcStructuralCurveMemberTypeEnum["userdefined"] = "userdefined";
    IfcStructuralCurveMemberTypeEnum["notdefined"] = "notdefined";
})(IfcStructuralCurveMemberTypeEnum || (IfcStructuralCurveMemberTypeEnum = {}));
var IfcStructuralSurfaceActivityTypeEnum;
(function (IfcStructuralSurfaceActivityTypeEnum) {
    IfcStructuralSurfaceActivityTypeEnum["const"] = "const";
    IfcStructuralSurfaceActivityTypeEnum["bilinear"] = "bilinear";
    IfcStructuralSurfaceActivityTypeEnum["discrete"] = "discrete";
    IfcStructuralSurfaceActivityTypeEnum["isocontour"] = "isocontour";
    IfcStructuralSurfaceActivityTypeEnum["userdefined"] = "userdefined";
    IfcStructuralSurfaceActivityTypeEnum["notdefined"] = "notdefined";
})(IfcStructuralSurfaceActivityTypeEnum || (IfcStructuralSurfaceActivityTypeEnum = {}));
var IfcStructuralSurfaceMemberTypeEnum;
(function (IfcStructuralSurfaceMemberTypeEnum) {
    IfcStructuralSurfaceMemberTypeEnum["bending_element"] = "bending_element";
    IfcStructuralSurfaceMemberTypeEnum["membrane_element"] = "membrane_element";
    IfcStructuralSurfaceMemberTypeEnum["shell"] = "shell";
    IfcStructuralSurfaceMemberTypeEnum["userdefined"] = "userdefined";
    IfcStructuralSurfaceMemberTypeEnum["notdefined"] = "notdefined";
})(IfcStructuralSurfaceMemberTypeEnum || (IfcStructuralSurfaceMemberTypeEnum = {}));
var IfcSubContractResourceTypeEnum;
(function (IfcSubContractResourceTypeEnum) {
    IfcSubContractResourceTypeEnum["purchase"] = "purchase";
    IfcSubContractResourceTypeEnum["work"] = "work";
    IfcSubContractResourceTypeEnum["userdefined"] = "userdefined";
    IfcSubContractResourceTypeEnum["notdefined"] = "notdefined";
})(IfcSubContractResourceTypeEnum || (IfcSubContractResourceTypeEnum = {}));
var IfcSurfaceFeatureTypeEnum;
(function (IfcSurfaceFeatureTypeEnum) {
    IfcSurfaceFeatureTypeEnum["mark"] = "mark";
    IfcSurfaceFeatureTypeEnum["tag"] = "tag";
    IfcSurfaceFeatureTypeEnum["treatment"] = "treatment";
    IfcSurfaceFeatureTypeEnum["userdefined"] = "userdefined";
    IfcSurfaceFeatureTypeEnum["notdefined"] = "notdefined";
})(IfcSurfaceFeatureTypeEnum || (IfcSurfaceFeatureTypeEnum = {}));
var IfcSurfaceSide;
(function (IfcSurfaceSide) {
    IfcSurfaceSide["positive"] = "positive";
    IfcSurfaceSide["negative"] = "negative";
    IfcSurfaceSide["both"] = "both";
})(IfcSurfaceSide || (IfcSurfaceSide = {}));
var IfcSwitchingDeviceTypeEnum;
(function (IfcSwitchingDeviceTypeEnum) {
    IfcSwitchingDeviceTypeEnum["contactor"] = "contactor";
    IfcSwitchingDeviceTypeEnum["dimmerswitch"] = "dimmerswitch";
    IfcSwitchingDeviceTypeEnum["emergencystop"] = "emergencystop";
    IfcSwitchingDeviceTypeEnum["keypad"] = "keypad";
    IfcSwitchingDeviceTypeEnum["momentaryswitch"] = "momentaryswitch";
    IfcSwitchingDeviceTypeEnum["selectorswitch"] = "selectorswitch";
    IfcSwitchingDeviceTypeEnum["starter"] = "starter";
    IfcSwitchingDeviceTypeEnum["switchdisconnector"] = "switchdisconnector";
    IfcSwitchingDeviceTypeEnum["toggleswitch"] = "toggleswitch";
    IfcSwitchingDeviceTypeEnum["userdefined"] = "userdefined";
    IfcSwitchingDeviceTypeEnum["notdefined"] = "notdefined";
})(IfcSwitchingDeviceTypeEnum || (IfcSwitchingDeviceTypeEnum = {}));
var IfcSystemFurnitureElementTypeEnum;
(function (IfcSystemFurnitureElementTypeEnum) {
    IfcSystemFurnitureElementTypeEnum["panel"] = "panel";
    IfcSystemFurnitureElementTypeEnum["worksurface"] = "worksurface";
    IfcSystemFurnitureElementTypeEnum["userdefined"] = "userdefined";
    IfcSystemFurnitureElementTypeEnum["notdefined"] = "notdefined";
})(IfcSystemFurnitureElementTypeEnum || (IfcSystemFurnitureElementTypeEnum = {}));
var IfcTankTypeEnum;
(function (IfcTankTypeEnum) {
    IfcTankTypeEnum["basin"] = "basin";
    IfcTankTypeEnum["breakpressure"] = "breakpressure";
    IfcTankTypeEnum["expansion"] = "expansion";
    IfcTankTypeEnum["feedandexpansion"] = "feedandexpansion";
    IfcTankTypeEnum["pressurevessel"] = "pressurevessel";
    IfcTankTypeEnum["storage"] = "storage";
    IfcTankTypeEnum["vessel"] = "vessel";
    IfcTankTypeEnum["userdefined"] = "userdefined";
    IfcTankTypeEnum["notdefined"] = "notdefined";
})(IfcTankTypeEnum || (IfcTankTypeEnum = {}));
var IfcTaskDurationEnum;
(function (IfcTaskDurationEnum) {
    IfcTaskDurationEnum["elapsedtime"] = "elapsedtime";
    IfcTaskDurationEnum["worktime"] = "worktime";
    IfcTaskDurationEnum["notdefined"] = "notdefined";
})(IfcTaskDurationEnum || (IfcTaskDurationEnum = {}));
var IfcTaskTypeEnum;
(function (IfcTaskTypeEnum) {
    IfcTaskTypeEnum["attendance"] = "attendance";
    IfcTaskTypeEnum["construction"] = "construction";
    IfcTaskTypeEnum["demolition"] = "demolition";
    IfcTaskTypeEnum["dismantle"] = "dismantle";
    IfcTaskTypeEnum["disposal"] = "disposal";
    IfcTaskTypeEnum["installation"] = "installation";
    IfcTaskTypeEnum["logistic"] = "logistic";
    IfcTaskTypeEnum["maintenance"] = "maintenance";
    IfcTaskTypeEnum["move"] = "move";
    IfcTaskTypeEnum["operation"] = "operation";
    IfcTaskTypeEnum["removal"] = "removal";
    IfcTaskTypeEnum["renovation"] = "renovation";
    IfcTaskTypeEnum["userdefined"] = "userdefined";
    IfcTaskTypeEnum["notdefined"] = "notdefined";
})(IfcTaskTypeEnum || (IfcTaskTypeEnum = {}));
var IfcTendonAnchorTypeEnum;
(function (IfcTendonAnchorTypeEnum) {
    IfcTendonAnchorTypeEnum["coupler"] = "coupler";
    IfcTendonAnchorTypeEnum["fixed_end"] = "fixed_end";
    IfcTendonAnchorTypeEnum["tensioning_end"] = "tensioning_end";
    IfcTendonAnchorTypeEnum["userdefined"] = "userdefined";
    IfcTendonAnchorTypeEnum["notdefined"] = "notdefined";
})(IfcTendonAnchorTypeEnum || (IfcTendonAnchorTypeEnum = {}));
var IfcTendonTypeEnum;
(function (IfcTendonTypeEnum) {
    IfcTendonTypeEnum["bar"] = "bar";
    IfcTendonTypeEnum["coated"] = "coated";
    IfcTendonTypeEnum["strand"] = "strand";
    IfcTendonTypeEnum["wire"] = "wire";
    IfcTendonTypeEnum["userdefined"] = "userdefined";
    IfcTendonTypeEnum["notdefined"] = "notdefined";
})(IfcTendonTypeEnum || (IfcTendonTypeEnum = {}));
var IfcTextPath;
(function (IfcTextPath) {
    IfcTextPath["left"] = "left";
    IfcTextPath["right"] = "right";
    IfcTextPath["up"] = "up";
    IfcTextPath["down"] = "down";
})(IfcTextPath || (IfcTextPath = {}));
var IfcTimeSeriesDataTypeEnum;
(function (IfcTimeSeriesDataTypeEnum) {
    IfcTimeSeriesDataTypeEnum["continuous"] = "continuous";
    IfcTimeSeriesDataTypeEnum["discrete"] = "discrete";
    IfcTimeSeriesDataTypeEnum["discretebinary"] = "discretebinary";
    IfcTimeSeriesDataTypeEnum["piecewisebinary"] = "piecewisebinary";
    IfcTimeSeriesDataTypeEnum["piecewiseconstant"] = "piecewiseconstant";
    IfcTimeSeriesDataTypeEnum["piecewisecontinuous"] = "piecewisecontinuous";
    IfcTimeSeriesDataTypeEnum["notdefined"] = "notdefined";
})(IfcTimeSeriesDataTypeEnum || (IfcTimeSeriesDataTypeEnum = {}));
var IfcTransformerTypeEnum;
(function (IfcTransformerTypeEnum) {
    IfcTransformerTypeEnum["current"] = "current";
    IfcTransformerTypeEnum["frequency"] = "frequency";
    IfcTransformerTypeEnum["inverter"] = "inverter";
    IfcTransformerTypeEnum["rectifier"] = "rectifier";
    IfcTransformerTypeEnum["voltage"] = "voltage";
    IfcTransformerTypeEnum["userdefined"] = "userdefined";
    IfcTransformerTypeEnum["notdefined"] = "notdefined";
})(IfcTransformerTypeEnum || (IfcTransformerTypeEnum = {}));
var IfcTransitionCode;
(function (IfcTransitionCode) {
    IfcTransitionCode["discontinuous"] = "discontinuous";
    IfcTransitionCode["continuous"] = "continuous";
    IfcTransitionCode["contsamegradient"] = "contsamegradient";
    IfcTransitionCode["contsamegradientsamecurvature"] = "contsamegradientsamecurvature";
})(IfcTransitionCode || (IfcTransitionCode = {}));
var IfcTransportElementTypeEnum;
(function (IfcTransportElementTypeEnum) {
    IfcTransportElementTypeEnum["elevator"] = "elevator";
    IfcTransportElementTypeEnum["escalator"] = "escalator";
    IfcTransportElementTypeEnum["movingwalkway"] = "movingwalkway";
    IfcTransportElementTypeEnum["craneway"] = "craneway";
    IfcTransportElementTypeEnum["liftinggear"] = "liftinggear";
    IfcTransportElementTypeEnum["userdefined"] = "userdefined";
    IfcTransportElementTypeEnum["notdefined"] = "notdefined";
})(IfcTransportElementTypeEnum || (IfcTransportElementTypeEnum = {}));
var IfcTrimmingPreference;
(function (IfcTrimmingPreference) {
    IfcTrimmingPreference["cartesian"] = "cartesian";
    IfcTrimmingPreference["parameter"] = "parameter";
    IfcTrimmingPreference["unspecified"] = "unspecified";
})(IfcTrimmingPreference || (IfcTrimmingPreference = {}));
var IfcTubeBundleTypeEnum;
(function (IfcTubeBundleTypeEnum) {
    IfcTubeBundleTypeEnum["finned"] = "finned";
    IfcTubeBundleTypeEnum["userdefined"] = "userdefined";
    IfcTubeBundleTypeEnum["notdefined"] = "notdefined";
})(IfcTubeBundleTypeEnum || (IfcTubeBundleTypeEnum = {}));
var IfcUnitEnum;
(function (IfcUnitEnum) {
    IfcUnitEnum["absorbeddoseunit"] = "absorbeddoseunit";
    IfcUnitEnum["amountofsubstanceunit"] = "amountofsubstanceunit";
    IfcUnitEnum["areaunit"] = "areaunit";
    IfcUnitEnum["doseequivalentunit"] = "doseequivalentunit";
    IfcUnitEnum["electriccapacitanceunit"] = "electriccapacitanceunit";
    IfcUnitEnum["electricchargeunit"] = "electricchargeunit";
    IfcUnitEnum["electricconductanceunit"] = "electricconductanceunit";
    IfcUnitEnum["electriccurrentunit"] = "electriccurrentunit";
    IfcUnitEnum["electricresistanceunit"] = "electricresistanceunit";
    IfcUnitEnum["electricvoltageunit"] = "electricvoltageunit";
    IfcUnitEnum["energyunit"] = "energyunit";
    IfcUnitEnum["forceunit"] = "forceunit";
    IfcUnitEnum["frequencyunit"] = "frequencyunit";
    IfcUnitEnum["illuminanceunit"] = "illuminanceunit";
    IfcUnitEnum["inductanceunit"] = "inductanceunit";
    IfcUnitEnum["lengthunit"] = "lengthunit";
    IfcUnitEnum["luminousfluxunit"] = "luminousfluxunit";
    IfcUnitEnum["luminousintensityunit"] = "luminousintensityunit";
    IfcUnitEnum["magneticfluxdensityunit"] = "magneticfluxdensityunit";
    IfcUnitEnum["magneticfluxunit"] = "magneticfluxunit";
    IfcUnitEnum["massunit"] = "massunit";
    IfcUnitEnum["planeangleunit"] = "planeangleunit";
    IfcUnitEnum["powerunit"] = "powerunit";
    IfcUnitEnum["pressureunit"] = "pressureunit";
    IfcUnitEnum["radioactivityunit"] = "radioactivityunit";
    IfcUnitEnum["solidangleunit"] = "solidangleunit";
    IfcUnitEnum["thermodynamictemperatureunit"] = "thermodynamictemperatureunit";
    IfcUnitEnum["timeunit"] = "timeunit";
    IfcUnitEnum["volumeunit"] = "volumeunit";
    IfcUnitEnum["userdefined"] = "userdefined";
})(IfcUnitEnum || (IfcUnitEnum = {}));
var IfcUnitaryControlElementTypeEnum;
(function (IfcUnitaryControlElementTypeEnum) {
    IfcUnitaryControlElementTypeEnum["alarmpanel"] = "alarmpanel";
    IfcUnitaryControlElementTypeEnum["controlpanel"] = "controlpanel";
    IfcUnitaryControlElementTypeEnum["gasdetectionpanel"] = "gasdetectionpanel";
    IfcUnitaryControlElementTypeEnum["indicatorpanel"] = "indicatorpanel";
    IfcUnitaryControlElementTypeEnum["mimicpanel"] = "mimicpanel";
    IfcUnitaryControlElementTypeEnum["humidistat"] = "humidistat";
    IfcUnitaryControlElementTypeEnum["thermostat"] = "thermostat";
    IfcUnitaryControlElementTypeEnum["weatherstation"] = "weatherstation";
    IfcUnitaryControlElementTypeEnum["userdefined"] = "userdefined";
    IfcUnitaryControlElementTypeEnum["notdefined"] = "notdefined";
})(IfcUnitaryControlElementTypeEnum || (IfcUnitaryControlElementTypeEnum = {}));
var IfcUnitaryEquipmentTypeEnum;
(function (IfcUnitaryEquipmentTypeEnum) {
    IfcUnitaryEquipmentTypeEnum["airhandler"] = "airhandler";
    IfcUnitaryEquipmentTypeEnum["airconditioningunit"] = "airconditioningunit";
    IfcUnitaryEquipmentTypeEnum["dehumidifier"] = "dehumidifier";
    IfcUnitaryEquipmentTypeEnum["splitsystem"] = "splitsystem";
    IfcUnitaryEquipmentTypeEnum["rooftopunit"] = "rooftopunit";
    IfcUnitaryEquipmentTypeEnum["userdefined"] = "userdefined";
    IfcUnitaryEquipmentTypeEnum["notdefined"] = "notdefined";
})(IfcUnitaryEquipmentTypeEnum || (IfcUnitaryEquipmentTypeEnum = {}));
var IfcValveTypeEnum;
(function (IfcValveTypeEnum) {
    IfcValveTypeEnum["airrelease"] = "airrelease";
    IfcValveTypeEnum["antivacuum"] = "antivacuum";
    IfcValveTypeEnum["changeover"] = "changeover";
    IfcValveTypeEnum["check"] = "check";
    IfcValveTypeEnum["commissioning"] = "commissioning";
    IfcValveTypeEnum["diverting"] = "diverting";
    IfcValveTypeEnum["drawoffcock"] = "drawoffcock";
    IfcValveTypeEnum["doublecheck"] = "doublecheck";
    IfcValveTypeEnum["doubleregulating"] = "doubleregulating";
    IfcValveTypeEnum["faucet"] = "faucet";
    IfcValveTypeEnum["flushing"] = "flushing";
    IfcValveTypeEnum["gascock"] = "gascock";
    IfcValveTypeEnum["gastap"] = "gastap";
    IfcValveTypeEnum["isolating"] = "isolating";
    IfcValveTypeEnum["mixing"] = "mixing";
    IfcValveTypeEnum["pressurereducing"] = "pressurereducing";
    IfcValveTypeEnum["pressurerelief"] = "pressurerelief";
    IfcValveTypeEnum["regulating"] = "regulating";
    IfcValveTypeEnum["safetycutoff"] = "safetycutoff";
    IfcValveTypeEnum["steamtrap"] = "steamtrap";
    IfcValveTypeEnum["stopcock"] = "stopcock";
    IfcValveTypeEnum["userdefined"] = "userdefined";
    IfcValveTypeEnum["notdefined"] = "notdefined";
})(IfcValveTypeEnum || (IfcValveTypeEnum = {}));
var IfcVibrationIsolatorTypeEnum;
(function (IfcVibrationIsolatorTypeEnum) {
    IfcVibrationIsolatorTypeEnum["compression"] = "compression";
    IfcVibrationIsolatorTypeEnum["spring"] = "spring";
    IfcVibrationIsolatorTypeEnum["userdefined"] = "userdefined";
    IfcVibrationIsolatorTypeEnum["notdefined"] = "notdefined";
})(IfcVibrationIsolatorTypeEnum || (IfcVibrationIsolatorTypeEnum = {}));
var IfcVoidingFeatureTypeEnum;
(function (IfcVoidingFeatureTypeEnum) {
    IfcVoidingFeatureTypeEnum["cutout"] = "cutout";
    IfcVoidingFeatureTypeEnum["notch"] = "notch";
    IfcVoidingFeatureTypeEnum["hole"] = "hole";
    IfcVoidingFeatureTypeEnum["miter"] = "miter";
    IfcVoidingFeatureTypeEnum["chamfer"] = "chamfer";
    IfcVoidingFeatureTypeEnum["edge"] = "edge";
    IfcVoidingFeatureTypeEnum["userdefined"] = "userdefined";
    IfcVoidingFeatureTypeEnum["notdefined"] = "notdefined";
})(IfcVoidingFeatureTypeEnum || (IfcVoidingFeatureTypeEnum = {}));
var IfcWallTypeEnum;
(function (IfcWallTypeEnum) {
    IfcWallTypeEnum["movable"] = "movable";
    IfcWallTypeEnum["parapet"] = "parapet";
    IfcWallTypeEnum["partitioning"] = "partitioning";
    IfcWallTypeEnum["plumbingwall"] = "plumbingwall";
    IfcWallTypeEnum["shear"] = "shear";
    IfcWallTypeEnum["solidwall"] = "solidwall";
    IfcWallTypeEnum["standard"] = "standard";
    IfcWallTypeEnum["polygonal"] = "polygonal";
    IfcWallTypeEnum["elementedwall"] = "elementedwall";
    IfcWallTypeEnum["userdefined"] = "userdefined";
    IfcWallTypeEnum["notdefined"] = "notdefined";
})(IfcWallTypeEnum || (IfcWallTypeEnum = {}));
var IfcWasteTerminalTypeEnum;
(function (IfcWasteTerminalTypeEnum) {
    IfcWasteTerminalTypeEnum["floortrap"] = "floortrap";
    IfcWasteTerminalTypeEnum["floorwaste"] = "floorwaste";
    IfcWasteTerminalTypeEnum["gullysump"] = "gullysump";
    IfcWasteTerminalTypeEnum["gullytrap"] = "gullytrap";
    IfcWasteTerminalTypeEnum["roofdrain"] = "roofdrain";
    IfcWasteTerminalTypeEnum["wastedisposalunit"] = "wastedisposalunit";
    IfcWasteTerminalTypeEnum["wastetrap"] = "wastetrap";
    IfcWasteTerminalTypeEnum["userdefined"] = "userdefined";
    IfcWasteTerminalTypeEnum["notdefined"] = "notdefined";
})(IfcWasteTerminalTypeEnum || (IfcWasteTerminalTypeEnum = {}));
var IfcWindowPanelOperationEnum;
(function (IfcWindowPanelOperationEnum) {
    IfcWindowPanelOperationEnum["sidehungrighthand"] = "sidehungrighthand";
    IfcWindowPanelOperationEnum["sidehunglefthand"] = "sidehunglefthand";
    IfcWindowPanelOperationEnum["tiltandturnrighthand"] = "tiltandturnrighthand";
    IfcWindowPanelOperationEnum["tiltandturnlefthand"] = "tiltandturnlefthand";
    IfcWindowPanelOperationEnum["tophung"] = "tophung";
    IfcWindowPanelOperationEnum["bottomhung"] = "bottomhung";
    IfcWindowPanelOperationEnum["pivothorizontal"] = "pivothorizontal";
    IfcWindowPanelOperationEnum["pivotvertical"] = "pivotvertical";
    IfcWindowPanelOperationEnum["slidinghorizontal"] = "slidinghorizontal";
    IfcWindowPanelOperationEnum["slidingvertical"] = "slidingvertical";
    IfcWindowPanelOperationEnum["removablecasement"] = "removablecasement";
    IfcWindowPanelOperationEnum["fixedcasement"] = "fixedcasement";
    IfcWindowPanelOperationEnum["otheroperation"] = "otheroperation";
    IfcWindowPanelOperationEnum["notdefined"] = "notdefined";
})(IfcWindowPanelOperationEnum || (IfcWindowPanelOperationEnum = {}));
var IfcWindowPanelPositionEnum;
(function (IfcWindowPanelPositionEnum) {
    IfcWindowPanelPositionEnum["left"] = "left";
    IfcWindowPanelPositionEnum["middle"] = "middle";
    IfcWindowPanelPositionEnum["right"] = "right";
    IfcWindowPanelPositionEnum["bottom"] = "bottom";
    IfcWindowPanelPositionEnum["top"] = "top";
    IfcWindowPanelPositionEnum["notdefined"] = "notdefined";
})(IfcWindowPanelPositionEnum || (IfcWindowPanelPositionEnum = {}));
var IfcWindowStyleConstructionEnum;
(function (IfcWindowStyleConstructionEnum) {
    IfcWindowStyleConstructionEnum["aluminium"] = "aluminium";
    IfcWindowStyleConstructionEnum["high_grade_steel"] = "high_grade_steel";
    IfcWindowStyleConstructionEnum["steel"] = "steel";
    IfcWindowStyleConstructionEnum["wood"] = "wood";
    IfcWindowStyleConstructionEnum["aluminium_wood"] = "aluminium_wood";
    IfcWindowStyleConstructionEnum["plastic"] = "plastic";
    IfcWindowStyleConstructionEnum["other_construction"] = "other_construction";
    IfcWindowStyleConstructionEnum["notdefined"] = "notdefined";
})(IfcWindowStyleConstructionEnum || (IfcWindowStyleConstructionEnum = {}));
var IfcWindowStyleOperationEnum;
(function (IfcWindowStyleOperationEnum) {
    IfcWindowStyleOperationEnum["single_panel"] = "single_panel";
    IfcWindowStyleOperationEnum["double_panel_vertical"] = "double_panel_vertical";
    IfcWindowStyleOperationEnum["double_panel_horizontal"] = "double_panel_horizontal";
    IfcWindowStyleOperationEnum["triple_panel_vertical"] = "triple_panel_vertical";
    IfcWindowStyleOperationEnum["triple_panel_bottom"] = "triple_panel_bottom";
    IfcWindowStyleOperationEnum["triple_panel_top"] = "triple_panel_top";
    IfcWindowStyleOperationEnum["triple_panel_left"] = "triple_panel_left";
    IfcWindowStyleOperationEnum["triple_panel_right"] = "triple_panel_right";
    IfcWindowStyleOperationEnum["triple_panel_horizontal"] = "triple_panel_horizontal";
    IfcWindowStyleOperationEnum["userdefined"] = "userdefined";
    IfcWindowStyleOperationEnum["notdefined"] = "notdefined";
})(IfcWindowStyleOperationEnum || (IfcWindowStyleOperationEnum = {}));
var IfcWindowTypeEnum;
(function (IfcWindowTypeEnum) {
    IfcWindowTypeEnum["window"] = "window";
    IfcWindowTypeEnum["skylight"] = "skylight";
    IfcWindowTypeEnum["lightdome"] = "lightdome";
    IfcWindowTypeEnum["userdefined"] = "userdefined";
    IfcWindowTypeEnum["notdefined"] = "notdefined";
})(IfcWindowTypeEnum || (IfcWindowTypeEnum = {}));
var IfcWindowTypePartitioningEnum;
(function (IfcWindowTypePartitioningEnum) {
    IfcWindowTypePartitioningEnum["single_panel"] = "single_panel";
    IfcWindowTypePartitioningEnum["double_panel_vertical"] = "double_panel_vertical";
    IfcWindowTypePartitioningEnum["double_panel_horizontal"] = "double_panel_horizontal";
    IfcWindowTypePartitioningEnum["triple_panel_vertical"] = "triple_panel_vertical";
    IfcWindowTypePartitioningEnum["triple_panel_bottom"] = "triple_panel_bottom";
    IfcWindowTypePartitioningEnum["triple_panel_top"] = "triple_panel_top";
    IfcWindowTypePartitioningEnum["triple_panel_left"] = "triple_panel_left";
    IfcWindowTypePartitioningEnum["triple_panel_right"] = "triple_panel_right";
    IfcWindowTypePartitioningEnum["triple_panel_horizontal"] = "triple_panel_horizontal";
    IfcWindowTypePartitioningEnum["userdefined"] = "userdefined";
    IfcWindowTypePartitioningEnum["notdefined"] = "notdefined";
})(IfcWindowTypePartitioningEnum || (IfcWindowTypePartitioningEnum = {}));
var IfcWorkCalendarTypeEnum;
(function (IfcWorkCalendarTypeEnum) {
    IfcWorkCalendarTypeEnum["firstshift"] = "firstshift";
    IfcWorkCalendarTypeEnum["secondshift"] = "secondshift";
    IfcWorkCalendarTypeEnum["thirdshift"] = "thirdshift";
    IfcWorkCalendarTypeEnum["userdefined"] = "userdefined";
    IfcWorkCalendarTypeEnum["notdefined"] = "notdefined";
})(IfcWorkCalendarTypeEnum || (IfcWorkCalendarTypeEnum = {}));
var IfcWorkPlanTypeEnum;
(function (IfcWorkPlanTypeEnum) {
    IfcWorkPlanTypeEnum["actual"] = "actual";
    IfcWorkPlanTypeEnum["baseline"] = "baseline";
    IfcWorkPlanTypeEnum["planned"] = "planned";
    IfcWorkPlanTypeEnum["userdefined"] = "userdefined";
    IfcWorkPlanTypeEnum["notdefined"] = "notdefined";
})(IfcWorkPlanTypeEnum || (IfcWorkPlanTypeEnum = {}));
var IfcWorkScheduleTypeEnum;
(function (IfcWorkScheduleTypeEnum) {
    IfcWorkScheduleTypeEnum["actual"] = "actual";
    IfcWorkScheduleTypeEnum["baseline"] = "baseline";
    IfcWorkScheduleTypeEnum["planned"] = "planned";
    IfcWorkScheduleTypeEnum["userdefined"] = "userdefined";
    IfcWorkScheduleTypeEnum["notdefined"] = "notdefined";
})(IfcWorkScheduleTypeEnum || (IfcWorkScheduleTypeEnum = {}));
//# sourceMappingURL=index.js.map