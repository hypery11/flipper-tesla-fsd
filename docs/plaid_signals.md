# Plaid drive-unit CAN signals (Model S/X, refreshed 2021+)

Every frame the four drive inverters send, as listed in Tesla's own Model S/X dictionary
(`can_frames_decoded_all_values_modelsx_amd.json`, firmware 2026.2.3, AMD / MCU3, the computer in refreshed
cars; [tesla-can-explorer](https://github.com/bruvv/tesla-can-explorer), extracted from `libQtCarVAPI.so`).

**What this list is and isn't**
- It gives the CAN ID, frame name and signal names. It does **not** give bit positions, scale or units.
  Only the rows marked *decoded* have a layout, borrowed from the Model 3/Y community DBC
  ([joshwardell/model3dbc](https://github.com/joshwardell/model3dbc)) where the signal set matches.
- The dictionary is the computer's "ETH" view. Which physical bus or harness pin a frame appears on is not
  documented. Chassis-bus frames such as wheel speeds (0x175) are not in it at all.
- A Plaid has DIF + DIREL + DIRER. A dual-motor Long Range has DIF + DIR.
- There is no oil *pressure* sensor signal. Pressure only appears as alert reasons and limp reasons.

## Frame index

| Frame | Front (DIF) | Rear-left (DIREL) | Rear-right (DIRER) | Rear (DIR) |
|---|---|---|---|---|
| alertLog | 0x726 | 0x5A7 | 0x5AA | 0x5A5 |
| alertMatrix1 | 0x356 | 0x3E6 | 0x3EC | 0x3A5 |
| alertMatrix2 | 0x357 | 0x3E7 | 0x3EE | 0x3B5 |
| alertMatrix3 | 0x35A | 0x3EA | 0x3EF | 0x3C5 |
| alertMatrix4 | 0x35B | 0x3EB | 0x3F0 | 0x3E5 |
| hvStatus | 0x27A | 0x27B | 0x27E | 0x279 |
| info | 0x656 | 0x337 | 0x342 | 0x335 |
| motorStatus | 0x1A5 | 0x7EF | 0x12A | 0x7EC |
| status | 0x2D5 | 0x7EE | 0x25A | 0x256 |
| temperature | 0x376 | 0x317 | 0x322 | 0x377 |
| thermalControl | 0x7F3 | 0x5DA | 0x5DB | 0x5D7 |
| torque | 0x186 | 0x188 | 0x10A | 0x108 |

## Front motor — `DIF`

### 0x186 `DIF_torque` — 9 signals
*Decoded:* `*_torqueActual` bit 27, 13 bits signed, ×2 Nm · `*_axleSpeed` bit 40, 16 bits signed, ×0.1 RPM (Model 3/Y DBC; decoded)

`DIF_axlePowerCalcQF`, `DIF_axleSpeed`, `DIF_axleSpeedQF`, `DIF_axleTorqueQF`, `DIF_gear`, `DIF_torqueActual`, `DIF_torqueChecksum`, `DIF_torqueCommand`, `DIF_torqueCounter`

### 0x1A5 `DIF_motorStatus` — 7 signals
*Decoded:* `*_motorCurrent` bit 11, 11 bits, ×1 A (from the Model 3/Y DBC; decoded on the Flipper Live Data screen)

`DIF_keepAliveRequest`, `DIF_motorCurrent`, `DIF_motorStatusChecksum`, `DIF_motorStatusCounter`, `DIF_switchShortTestRetryCount`, `DIF_switchingFrequency`, `DIF_targetFluxMode`

### 0x27A `DIF_hvStatus` — 15 signals
`DIF_activeDischargeCtrl`, `DIF_activeDischargeStatus`, `DIF_fluxState`, `DIF_gateDriveState`, `DIF_gateDriveSupplyState`, `DIF_hvStatusChecksum`, `DIF_hvStatusCounter`, `DIF_hvilCmVoltage`, `DIF_hvilCurrent`, `DIF_hvilStatus`, `DIF_hvilSupplyState`, `DIF_powerStageSafeState`, `DIF_vBat`, `DIF_vBatQF`, `DIF_vBusDiag`

### 0x2D5 `DIF_status` — 16 signals
`DIF_adDirection`, `DIF_adSpeedDiff`, `DIF_adState`, `DIF_axleSpeedLimitRequest`, `DIF_driveModeState`, `DIF_haltRequest`, `DIF_lvSupplyV`, `DIF_softSysLimpRequest`, `DIF_soptState`, `DIF_spinDownLearningMode`, `DIF_spinDownLearningState`, `DIF_state`, `DIF_statusChecksum`, `DIF_statusCounter`, `DIF_sysLimpRequest`, `DIF_wasteState`

### 0x356 `DIF_alertMatrix1` — 61 signals
`DIF_a001_hwPhaseAgateDrive`, `DIF_a002_hwPhaseBgateDrive`, `DIF_a003_hwPhaseCgateDrive`, `DIF_a004_hwPhaseApeak`, `DIF_a005_hwPhaseBpeak`, `DIF_a006_hwPhaseCpeak`, `DIF_a007_statorAnomalyDetected`, `DIF_a008_hwEncoderA`, `DIF_a009_hwEncoderB`, `DIF_a010_unintendedReset`, `DIF_a011_swPowerStageNotReady`, `DIF_a012_hvilNotClosed`, `DIF_a013_eccError`, `DIF_a014_activeDamping`, `DIF_a015_mechSafeStateAnomaly`, `DIF_a016_safeStateApplied`, `DIF_a017_hwPedalMonitor`, `DIF_a018_hwLVSupplyUV`, `DIF_a020_hwMotorEncoder`, `DIF_a021_hwBusOV`, `DIF_a022_hw5vSupplyUV`, `DIF_a023_clutchPositionIrrational`, `DIF_a024_selfTest`, `DIF_a025_phaseApeak`, `DIF_a026_phaseBpeak`, `DIF_a027_phaseCpeak`, `DIF_a028_phaseArms`, `DIF_a029_phaseBrms`, `DIF_a030_phaseCrms`, `DIF_a031_phaseAcurrentOffset`, `DIF_a032_phaseBcurrentOffset`, `DIF_a033_phaseAcurrentSensor`, `DIF_a034_phaseBcurrentSensor`, `DIF_a035_phaseCurrentBalance`, `DIF_a036_busOV`, `DIF_a037_busUV`, `DIF_a038_busVsensor`, `DIF_a039_clutchDriverFault`, `DIF_a040_difMIA`, `DIF_a041_sdcMIA`, `DIF_a042_inletSensor`, `DIF_a043_outletOT`, `DIF_a044_outletUT`, `DIF_a045_outletSensor`, `DIF_a046_statorOT`, `DIF_a047_statorSensor1`, `DIF_a048_statorSensor2`, `DIF_a049_statorSensorDiff`, `DIF_a050_noStatorSensor`, `DIF_a052_dirMIA`, `DIF_a053_unexpectedLatchState`, `DIF_a054_driveInverterOT`, `DIF_a055_trqCrossCheck`, `DIF_a056_ambientOT`, `DIF_a057_ambientUT`, `DIF_a058_ambientSensor`, `DIF_a059_heatsinkOT`, `DIF_a060_heatsinkUT`, `DIF_a061_heatsinkSensor`, `DIF_a062_systemLimpMode`, `DIF_a064_torqueIntervention`

### 0x357 `DIF_alertMatrix2` — 55 signals
`DIF_a065_canHardwareBusB`, `DIF_a066_canDataBusB`, `DIF_a067_canOverrunBusB`, `DIF_a068_ccbMIA`, `DIF_a069_rotorTempLimit`, `DIF_a070_udsTransactionInitiated`, `DIF_a071_busDisconnected`, `DIF_a072_gateDriveFaultCounter`, `DIF_a073_motorSpeed`, `DIF_a074_motorEncoder`, `DIF_a075_motorSpeedMismatch`, `DIF_a076_lvSupplyOV`, `DIF_a077_lvSupplyUV`, `DIF_a078_adcRefLow`, `DIF_a079_adcRefHigh`, `DIF_a084_eccTestData0`, `DIF_a086_activeDischargeOn`, `DIF_a087_gtwMIA`, `DIF_a088_uiMIA`, `DIF_a090_pmMIA`, `DIF_a091_espMIA`, `DIF_a092_bmsMIA`, `DIF_a093_canHardwareBusA`, `DIF_a094_canDataBusA`, `DIF_a095_canOverrunBusA`, `DIF_a096_memoryError`, `DIF_a097_eepromError`, `DIF_a098_intTimeTooLong`, `DIF_a099_threadOverrun`, `DIF_a100_assertion`, `DIF_a101_eccTestData1`, `DIF_a103_lowFlow`, `DIF_a107_fpgaError`, `DIF_a108_stateTrans`, `DIF_a110_brakeMIA`, `DIF_a111_badPhaseSensorCalib`, `DIF_a112_noPhaseCurrent`, `DIF_a113_noFuncHeatsinkSensor`, `DIF_a116_highSpeedWearCounter`, `DIF_a117_diMIA`, `DIF_a119_hvpMIA`, `DIF_a120_hvlinkMIA`, `DIF_a121_motorControlRegulation`, `DIF_a122_highStackUsage`, `DIF_a123_lossMotorControl`, `DIF_a126_limpMode`, `DIF_a127_gracefulPowerOff`, `DIF_a128_fpgaVersionMismatch`, `DIF_d085_driveInverterBoardFailure`, `DIF_d089_invalidTorqueCommand`, `DIF_d104_resetUnintended`, `DIF_d105_busVoltageSensorIssue`, `DIF_d118_oilPumpCommsLost`, `DIF_d124_oilPumpServiceRequired`, `DIF_d125_currentSensorIssue`

### 0x35A `DIF_alertMatrix3` — 40 signals
`DIF_a133_capacitorOT`, `DIF_a134_wheelSpeedIrrational`, `DIF_a136_spiError`, `DIF_a142_highLashAngle`, `DIF_a144_configMismatch`, `DIF_a147_highTorqueWearLimit`, `DIF_a148_burnInCycleEnded`, `DIF_a149_oilPumpFailure`, `DIF_a150_busVD`, `DIF_a151_shockTorqueLimiter`, `DIF_a152_linError`, `DIF_a153_oilPumpDiagnostics`, `DIF_a154_resolver`, `DIF_a155_vcfrontMIA`, `DIF_a156_currentObserver`, `DIF_a157_rcmMIA`, `DIF_a158_ibstMIA`, `DIF_a160_busVoltageAnomaly`, `DIF_a161_epas3pMIA`, `DIF_a162_endOfLifetimeBurnIn`, `DIF_a172_statorOilTempOT`, `DIF_a173_ascFdbkDiagnostic`, `DIF_a174_unitMayNotRestart`, `DIF_a176_safetyICFault`, `DIF_a177_fluxReferenceCorrected`, `DIF_d129_oilPumpIssue`, `DIF_d130_positionSensorIssue`, `DIF_d131_currentSensorOffset`, `DIF_d132_busUVIssue`, `DIF_d135_rotatedStator`, `DIF_d137_currentSensorOutOfRange`, `DIF_d138_inverterLVPowerSupplyLow`, `DIF_d139_unitEEPROMIssue`, `DIF_d141_coolingSystemIssue`, `DIF_d143_wheelSpeedCorrelationIssue`, `DIF_d163_recoverableOvercurrent`, `DIF_d164_firmwareConfigMismatch`, `DIF_d165_statorOverTemperature`, `DIF_d166_rotorOverTemperature`, `DIF_d167_tempSensorIssue`

### 0x35B `DIF_alertMatrix4` — 20 signals
`DIF_a202_excessHeatUnavailable`, `DIF_a225_tmpEstPlausibility`, `DIF_a229_busVsensorOverCAN`, `DIF_a234_pcsMIA`, `DIF_a236_tasMIA`, `DIF_a240_clearFusesRoutine`, `DIF_a241_systemThermallyLimited`, `DIF_a242_spinDownLearning`, `DIF_a243_ecuLogAvailable`, `DIF_a244_mechSafeStateApplied`, `DIF_a245_mechSafeStatePreWarn`, `DIF_a246_recoveryAtSpeedError`, `DIF_a247_rotorOffsetError`, `DIF_a249_highStatorTempFromResist`, `DIF_a250_preregulatorRail`, `DIF_a251_oilPumpService`, `DIF_a252_currentCoreFallback`, `DIF_a253_lvBoostedRail`, `DIF_a254_activeDischargeRail`, `DIF_a255_currentGainFallback`

### 0x376 `DIF_temperature` — 21 signals
`DIF_tempIndex`, `DIF_inverterTQF`, `DIF_pcbT`, `DIF_inverterT`, `DIF_statorT`, `DIF_dcCapT`, `DIF_heatsinkT`, `DIF_inverterTpct`, `DIF_statorTpct`, `DIF_fluidInTemp`, `DIF_normalFetBurnIn`, `DIF_additionalFetBurnIn`, `DIF_currentWeibullMiles`, `DIF_endOfServiceWeibullMiles`, `DIF_burnInDamageRatio`, `DIF_inverterSensorEst`, `DIF_inverterHS1Est`, `DIF_initialBurnInVehicleOdometer`, `DIF_inverterHS2Est`, `DIF_inverterInletEst`, `DIF_statorHousingTemp`

### 0x656 `DIF_info` — 25 signals
`DIF_infoIndex`, `DIF_buildType`, `DIF_buildConfigurationId`, `DIF_hardwareId`, `DIF_componentId`, `DIF_pcbaId`, `DIF_assemblyId`, `DIF_usageId`, `DIF_subUsageId`, `DIF_motorType`, `DIF_siliconType`, `DIF_mechSafeStateType`, `DIF_applicationCrc`, `DIF_oilPumpBuildType`, `DIF_oilPumpAppCrc`, `DIF_FPGA_version`, `DIF_appGitHash`, `DIF_bootGitHash`, `DIF_oilPumpPcbaId`, `DIF_oilPumpAssemblyId`, `DIF_oilPumpUsageId`, `DIF_refRotorFlux`, `DIF_refRotorOffset`, `DIF_speedWearTotal`, `DIF_torqueWearTotal`

### 0x726 `DIF_alertLog` — 588 signals
Alert snapshot frame (588 signals, one group per alert). Oil-related: `DIF_a144_oilPump1HwType`, `DIF_a144_oilPump2HwType`, `DIF_a251_oilFlow`, `DIF_a251_oilTemperature`

### 0x7F3 `DIF_thermalControl` — 6 signals
`DIF_activeInletTempReq`, `DIF_burnInStatus`, `DIF_coolantFlowReq`, `DIF_criticalFlowReq`, `DIF_oilFlowReq`, `DIF_passiveInletTempReq`

## Rear-left motor (Plaid) — `DIREL`

### 0x188 `DIREL_torque` — 9 signals
*Decoded:* `*_torqueActual` bit 27, 13 bits signed, ×2 Nm · `*_axleSpeed` bit 40, 16 bits signed, ×0.1 RPM (Model 3/Y DBC; decoded)

`DIREL_axlePowerCalcQF`, `DIREL_axleSpeed`, `DIREL_axleSpeedQF`, `DIREL_axleTorqueQF`, `DIREL_gear`, `DIREL_torqueActual`, `DIREL_torqueChecksum`, `DIREL_torqueCommand`, `DIREL_torqueCounter`

### 0x27B `DIREL_hvStatus` — 15 signals
`DIREL_activeDischargeCtrl`, `DIREL_activeDischargeStatus`, `DIREL_fluxState`, `DIREL_gateDriveState`, `DIREL_gateDriveSupplyState`, `DIREL_hvStatusChecksum`, `DIREL_hvStatusCounter`, `DIREL_hvilCmVoltage`, `DIREL_hvilCurrent`, `DIREL_hvilStatus`, `DIREL_hvilSupplyState`, `DIREL_powerStageSafeState`, `DIREL_vBat`, `DIREL_vBatQF`, `DIREL_vBusDiag`

### 0x317 `DIREL_temperature` — 21 signals
`DIREL_tempIndex`, `DIREL_inverterTQF`, `DIREL_pcbT`, `DIREL_inverterT`, `DIREL_statorT`, `DIREL_dcCapT`, `DIREL_heatsinkT`, `DIREL_inverterTpct`, `DIREL_statorTpct`, `DIREL_fluidInTemp`, `DIREL_normalFetBurnIn`, `DIREL_additionalFetBurnIn`, `DIREL_currentWeibullMiles`, `DIREL_endOfServiceWeibullMiles`, `DIREL_burnInDamageRatio`, `DIREL_inverterSensorEst`, `DIREL_inverterHS1Est`, `DIREL_initialBurnInVehicleOdometer`, `DIREL_inverterHS2Est`, `DIREL_inverterInletEst`, `DIREL_statorHousingTemp`

### 0x337 `DIREL_info` — 30 signals
`DIREL_infoIndex`, `DIREL_buildType`, `DIREL_buildConfigurationId`, `DIREL_hardwareId`, `DIREL_componentId`, `DIREL_pcbaId`, `DIREL_assemblyId`, `DIREL_usageId`, `DIREL_subUsageId`, `DIREL_motorType`, `DIREL_siliconType`, `DIREL_mechSafeStateType`, `DIREL_applicationCrc`, `DIREL_oilPumpBuildType`, `DIREL_oilPumpAppCrc`, `DIREL_FPGA_version`, `DIREL_appGitHash`, `DIREL_bootGitHash`, `DIREL_oilPump2BuildType`, `DIREL_oilPump2AppCrc`, `DIREL_oilPumpPcbaId`, `DIREL_oilPumpAssemblyId`, `DIREL_oilPumpUsageId`, `DIREL_oilPump2PcbaId`, `DIREL_oilPump2AssemblyId`, `DIREL_oilPump2UsageId`, `DIREL_refRotorFlux`, `DIREL_refRotorOffset`, `DIREL_speedWearTotal`, `DIREL_torqueWearTotal`

### 0x3E6 `DIREL_alertMatrix1` — 57 signals
`DIREL_a001_hwPhaseAgateDrive`, `DIREL_a002_hwPhaseBgateDrive`, `DIREL_a003_hwPhaseCgateDrive`, `DIREL_a004_hwPhaseApeak`, `DIREL_a005_hwPhaseBpeak`, `DIREL_a006_hwPhaseCpeak`, `DIREL_a007_statorAnomalyDetected`, `DIREL_a008_hwEncoderA`, `DIREL_a009_hwEncoderB`, `DIREL_a010_unintendedReset`, `DIREL_a011_swPowerStageNotReady`, `DIREL_a012_hvilNotClosed`, `DIREL_a013_eccError`, `DIREL_a014_activeDamping`, `DIREL_a015_mechSafeStateAnomaly`, `DIREL_a016_safeStateApplied`, `DIREL_a017_hwPedalMonitor`, `DIREL_a018_hwLVSupplyUV`, `DIREL_a020_hwMotorEncoder`, `DIREL_a021_hwBusOV`, `DIREL_a022_hw5vSupplyUV`, `DIREL_a024_selfTest`, `DIREL_a025_phaseApeak`, `DIREL_a026_phaseBpeak`, `DIREL_a027_phaseCpeak`, `DIREL_a028_phaseArms`, `DIREL_a029_phaseBrms`, `DIREL_a030_phaseCrms`, `DIREL_a031_phaseAcurrentOffset`, `DIREL_a032_phaseBcurrentOffset`, `DIREL_a033_phaseAcurrentSensor`, `DIREL_a034_phaseBcurrentSensor`, `DIREL_a035_phaseCurrentBalance`, `DIREL_a036_busOV`, `DIREL_a037_busUV`, `DIREL_a038_busVsensor`, `DIREL_a041_sdcMIA`, `DIREL_a042_inletSensor`, `DIREL_a043_outletOT`, `DIREL_a044_outletUT`, `DIREL_a045_outletSensor`, `DIREL_a046_statorOT`, `DIREL_a047_statorSensor1`, `DIREL_a048_statorSensor2`, `DIREL_a049_statorSensorDiff`, `DIREL_a050_noStatorSensor`, `DIREL_a053_unexpectedLatchState`, `DIREL_a054_driveInverterOT`, `DIREL_a055_trqCrossCheck`, `DIREL_a056_ambientOT`, `DIREL_a057_ambientUT`, `DIREL_a058_ambientSensor`, `DIREL_a059_heatsinkOT`, `DIREL_a060_heatsinkUT`, `DIREL_a061_heatsinkSensor`, `DIREL_a062_systemLimpMode`, `DIREL_a064_torqueIntervention`

### 0x3E7 `DIREL_alertMatrix2` — 54 signals
`DIREL_a065_canHardwareBusB`, `DIREL_a066_canDataBusB`, `DIREL_a067_canOverrunBusB`, `DIREL_a069_rotorTempLimit`, `DIREL_a070_udsTransactionInitiated`, `DIREL_a071_busDisconnected`, `DIREL_a072_gateDriveFaultCounter`, `DIREL_a073_motorSpeed`, `DIREL_a074_motorEncoder`, `DIREL_a075_motorSpeedMismatch`, `DIREL_a076_lvSupplyOV`, `DIREL_a077_lvSupplyUV`, `DIREL_a078_adcRefLow`, `DIREL_a079_adcRefHigh`, `DIREL_a084_eccTestData0`, `DIREL_a086_activeDischargeOn`, `DIREL_a087_gtwMIA`, `DIREL_a088_uiMIA`, `DIREL_a090_pmMIA`, `DIREL_a091_espMIA`, `DIREL_a092_bmsMIA`, `DIREL_a093_canHardwareBusA`, `DIREL_a094_canDataBusA`, `DIREL_a095_canOverrunBusA`, `DIREL_a096_memoryError`, `DIREL_a097_eepromError`, `DIREL_a098_intTimeTooLong`, `DIREL_a099_threadOverrun`, `DIREL_a100_assertion`, `DIREL_a101_eccTestData1`, `DIREL_a103_lowFlow`, `DIREL_a107_fpgaError`, `DIREL_a108_stateTrans`, `DIREL_a110_brakeMIA`, `DIREL_a111_badPhaseSensorCalib`, `DIREL_a112_noPhaseCurrent`, `DIREL_a113_noFuncHeatsinkSensor`, `DIREL_a116_highSpeedWearCounter`, `DIREL_a117_diMIA`, `DIREL_a119_hvpMIA`, `DIREL_a120_hvlinkMIA`, `DIREL_a121_motorControlRegulation`, `DIREL_a122_highStackUsage`, `DIREL_a123_lossMotorControl`, `DIREL_a126_limpMode`, `DIREL_a127_gracefulPowerOff`, `DIREL_a128_fpgaVersionMismatch`, `DIREL_d085_driveInverterBoardFailure`, `DIREL_d089_invalidTorqueCommand`, `DIREL_d104_resetUnintended`, `DIREL_d105_busVoltageSensorIssue`, `DIREL_d118_oilPumpCommsLost`, `DIREL_d124_oilPumpServiceRequired`, `DIREL_d125_currentSensorIssue`

### 0x3EA `DIREL_alertMatrix3` — 41 signals
`DIREL_a133_capacitorOT`, `DIREL_a134_wheelSpeedIrrational`, `DIREL_a136_spiError`, `DIREL_a142_highLashAngle`, `DIREL_a144_configMismatch`, `DIREL_a147_highTorqueWearLimit`, `DIREL_a148_burnInCycleEnded`, `DIREL_a149_oilPumpScavengeFail`, `DIREL_a150_busVD`, `DIREL_a151_shockTorqueLimiter`, `DIREL_a152_linError`, `DIREL_a153_oilPumpDiagnostics`, `DIREL_a154_resolver`, `DIREL_a155_vcfrontMIA`, `DIREL_a156_currentObserver`, `DIREL_a157_rcmMIA`, `DIREL_a158_ibstMIA`, `DIREL_a159_oilPumpFailure`, `DIREL_a160_busVoltageAnomaly`, `DIREL_a161_epas3pMIA`, `DIREL_a162_endOfLifetimeBurnIn`, `DIREL_a172_statorOilTempOT`, `DIREL_a173_ascFdbkDiagnostic`, `DIREL_a174_unitMayNotRestart`, `DIREL_a176_safetyICFault`, `DIREL_a177_fluxReferenceCorrected`, `DIREL_d129_oilPumpIssue`, `DIREL_d130_positionSensorIssue`, `DIREL_d131_currentSensorOffset`, `DIREL_d132_busUVIssue`, `DIREL_d135_rotatedStator`, `DIREL_d137_currentSensorOutOfRange`, `DIREL_d138_inverterLVPowerSupplyLow`, `DIREL_d139_unitEEPROMIssue`, `DIREL_d141_coolingSystemIssue`, `DIREL_d143_wheelSpeedCorrelationIssue`, `DIREL_d163_recoverableOvercurrent`, `DIREL_d164_firmwareConfigMismatch`, `DIREL_d165_statorOverTemperature`, `DIREL_d166_rotorOverTemperature`, `DIREL_d167_tempSensorIssue`

### 0x3EB `DIREL_alertMatrix4` — 20 signals
`DIREL_a202_excessHeatUnavailable`, `DIREL_a225_tmpEstPlausibility`, `DIREL_a229_busVsensorOverCAN`, `DIREL_a234_pcsMIA`, `DIREL_a236_tasMIA`, `DIREL_a240_clearFusesRoutine`, `DIREL_a241_systemThermallyLimited`, `DIREL_a242_spinDownLearning`, `DIREL_a243_ecuLogAvailable`, `DIREL_a244_mechSafeStateApplied`, `DIREL_a245_mechSafeStatePreWarn`, `DIREL_a246_recoveryAtSpeedError`, `DIREL_a247_rotorOffsetError`, `DIREL_a249_highStatorTempFromResist`, `DIREL_a250_preregulatorRail`, `DIREL_a251_oilPumpService`, `DIREL_a252_currentCoreFallback`, `DIREL_a253_lvBoostedRail`, `DIREL_a254_activeDischargeRail`, `DIREL_a255_currentGainFallback`

### 0x5A7 `DIREL_alertLog` — 573 signals
Alert snapshot frame (573 signals, one group per alert). Oil-related: `DIREL_a144_oilPump1HwType`, `DIREL_a144_oilPump2HwType`, `DIREL_a159_oilPumpAlert`, `DIREL_a251_oilFlow`, `DIREL_a251_oilTemperature`

### 0x5DA `DIREL_thermalControl` — 6 signals
`DIREL_activeInletTempReq`, `DIREL_burnInStatus`, `DIREL_coolantFlowReq`, `DIREL_criticalFlowReq`, `DIREL_oilFlowReq`, `DIREL_passiveInletTempReq`

### 0x7EE `DIREL_status` — 16 signals
`DIREL_adDirection`, `DIREL_adSpeedDiff`, `DIREL_adState`, `DIREL_axleSpeedLimitRequest`, `DIREL_driveModeState`, `DIREL_haltRequest`, `DIREL_lvSupplyV`, `DIREL_softSysLimpRequest`, `DIREL_soptState`, `DIREL_spinDownLearningMode`, `DIREL_spinDownLearningState`, `DIREL_state`, `DIREL_statusChecksum`, `DIREL_statusCounter`, `DIREL_sysLimpRequest`, `DIREL_wasteState`

### 0x7EF `DIREL_motorStatus` — 7 signals
*Decoded:* `*_motorCurrent` bit 11, 11 bits, ×1 A (from the Model 3/Y DBC; decoded on the Flipper Live Data screen)

`DIREL_keepAliveRequest`, `DIREL_motorCurrent`, `DIREL_motorStatusChecksum`, `DIREL_motorStatusCounter`, `DIREL_switchShortTestRetryCount`, `DIREL_switchingFrequency`, `DIREL_targetFluxMode`

## Rear-right motor (Plaid) — `DIRER`

### 0x10A `DIRER_torque` — 9 signals
*Decoded:* `*_torqueActual` bit 27, 13 bits signed, ×2 Nm · `*_axleSpeed` bit 40, 16 bits signed, ×0.1 RPM (Model 3/Y DBC; decoded)

`DIRER_axlePowerCalcQF`, `DIRER_axleSpeed`, `DIRER_axleSpeedQF`, `DIRER_axleTorqueQF`, `DIRER_gear`, `DIRER_torqueActual`, `DIRER_torqueChecksum`, `DIRER_torqueCommand`, `DIRER_torqueCounter`

### 0x12A `DIRER_motorStatus` — 7 signals
*Decoded:* `*_motorCurrent` bit 11, 11 bits, ×1 A (from the Model 3/Y DBC; decoded on the Flipper Live Data screen)

`DIRER_keepAliveRequest`, `DIRER_motorCurrent`, `DIRER_motorStatusChecksum`, `DIRER_motorStatusCounter`, `DIRER_switchShortTestRetryCount`, `DIRER_switchingFrequency`, `DIRER_targetFluxMode`

### 0x25A `DIRER_status` — 16 signals
`DIRER_adDirection`, `DIRER_adSpeedDiff`, `DIRER_adState`, `DIRER_axleSpeedLimitRequest`, `DIRER_driveModeState`, `DIRER_haltRequest`, `DIRER_lvSupplyV`, `DIRER_softSysLimpRequest`, `DIRER_soptState`, `DIRER_spinDownLearningMode`, `DIRER_spinDownLearningState`, `DIRER_state`, `DIRER_statusChecksum`, `DIRER_statusCounter`, `DIRER_sysLimpRequest`, `DIRER_wasteState`

### 0x27E `DIRER_hvStatus` — 15 signals
`DIRER_activeDischargeCtrl`, `DIRER_activeDischargeStatus`, `DIRER_fluxState`, `DIRER_gateDriveState`, `DIRER_gateDriveSupplyState`, `DIRER_hvStatusChecksum`, `DIRER_hvStatusCounter`, `DIRER_hvilCmVoltage`, `DIRER_hvilCurrent`, `DIRER_hvilStatus`, `DIRER_hvilSupplyState`, `DIRER_powerStageSafeState`, `DIRER_vBat`, `DIRER_vBatQF`, `DIRER_vBusDiag`

### 0x322 `DIRER_temperature` — 21 signals
`DIRER_tempIndex`, `DIRER_inverterTQF`, `DIRER_pcbT`, `DIRER_inverterT`, `DIRER_statorT`, `DIRER_dcCapT`, `DIRER_heatsinkT`, `DIRER_inverterTpct`, `DIRER_statorTpct`, `DIRER_fluidInTemp`, `DIRER_normalFetBurnIn`, `DIRER_additionalFetBurnIn`, `DIRER_currentWeibullMiles`, `DIRER_endOfServiceWeibullMiles`, `DIRER_burnInDamageRatio`, `DIRER_inverterSensorEst`, `DIRER_inverterHS1Est`, `DIRER_initialBurnInVehicleOdometer`, `DIRER_inverterHS2Est`, `DIRER_inverterInletEst`, `DIRER_statorHousingTemp`

### 0x342 `DIRER_info` — 25 signals
`DIRER_infoIndex`, `DIRER_buildType`, `DIRER_buildConfigurationId`, `DIRER_hardwareId`, `DIRER_componentId`, `DIRER_pcbaId`, `DIRER_assemblyId`, `DIRER_usageId`, `DIRER_subUsageId`, `DIRER_motorType`, `DIRER_siliconType`, `DIRER_mechSafeStateType`, `DIRER_applicationCrc`, `DIRER_oilPumpBuildType`, `DIRER_oilPumpAppCrc`, `DIRER_FPGA_version`, `DIRER_appGitHash`, `DIRER_bootGitHash`, `DIRER_oilPumpPcbaId`, `DIRER_oilPumpAssemblyId`, `DIRER_oilPumpUsageId`, `DIRER_refRotorFlux`, `DIRER_refRotorOffset`, `DIRER_speedWearTotal`, `DIRER_torqueWearTotal`

### 0x3EC `DIRER_alertMatrix1` — 57 signals
`DIRER_a001_hwPhaseAgateDrive`, `DIRER_a002_hwPhaseBgateDrive`, `DIRER_a003_hwPhaseCgateDrive`, `DIRER_a004_hwPhaseApeak`, `DIRER_a005_hwPhaseBpeak`, `DIRER_a006_hwPhaseCpeak`, `DIRER_a007_statorAnomalyDetected`, `DIRER_a008_hwEncoderA`, `DIRER_a009_hwEncoderB`, `DIRER_a010_unintendedReset`, `DIRER_a011_swPowerStageNotReady`, `DIRER_a012_hvilNotClosed`, `DIRER_a013_eccError`, `DIRER_a014_activeDamping`, `DIRER_a015_mechSafeStateAnomaly`, `DIRER_a016_safeStateApplied`, `DIRER_a017_hwPedalMonitor`, `DIRER_a018_hwLVSupplyUV`, `DIRER_a020_hwMotorEncoder`, `DIRER_a021_hwBusOV`, `DIRER_a022_hw5vSupplyUV`, `DIRER_a024_selfTest`, `DIRER_a025_phaseApeak`, `DIRER_a026_phaseBpeak`, `DIRER_a027_phaseCpeak`, `DIRER_a028_phaseArms`, `DIRER_a029_phaseBrms`, `DIRER_a030_phaseCrms`, `DIRER_a031_phaseAcurrentOffset`, `DIRER_a032_phaseBcurrentOffset`, `DIRER_a033_phaseAcurrentSensor`, `DIRER_a034_phaseBcurrentSensor`, `DIRER_a035_phaseCurrentBalance`, `DIRER_a036_busOV`, `DIRER_a037_busUV`, `DIRER_a038_busVsensor`, `DIRER_a041_sdcMIA`, `DIRER_a042_inletSensor`, `DIRER_a043_outletOT`, `DIRER_a044_outletUT`, `DIRER_a045_outletSensor`, `DIRER_a046_statorOT`, `DIRER_a047_statorSensor1`, `DIRER_a048_statorSensor2`, `DIRER_a049_statorSensorDiff`, `DIRER_a050_noStatorSensor`, `DIRER_a053_unexpectedLatchState`, `DIRER_a054_driveInverterOT`, `DIRER_a055_trqCrossCheck`, `DIRER_a056_ambientOT`, `DIRER_a057_ambientUT`, `DIRER_a058_ambientSensor`, `DIRER_a059_heatsinkOT`, `DIRER_a060_heatsinkUT`, `DIRER_a061_heatsinkSensor`, `DIRER_a062_systemLimpMode`, `DIRER_a064_torqueIntervention`

### 0x3EE `DIRER_alertMatrix2` — 54 signals
`DIRER_a065_canHardwareBusB`, `DIRER_a066_canDataBusB`, `DIRER_a067_canOverrunBusB`, `DIRER_a069_rotorTempLimit`, `DIRER_a070_udsTransactionInitiated`, `DIRER_a071_busDisconnected`, `DIRER_a072_gateDriveFaultCounter`, `DIRER_a073_motorSpeed`, `DIRER_a074_motorEncoder`, `DIRER_a075_motorSpeedMismatch`, `DIRER_a076_lvSupplyOV`, `DIRER_a077_lvSupplyUV`, `DIRER_a078_adcRefLow`, `DIRER_a079_adcRefHigh`, `DIRER_a084_eccTestData0`, `DIRER_a086_activeDischargeOn`, `DIRER_a087_gtwMIA`, `DIRER_a088_uiMIA`, `DIRER_a090_pmMIA`, `DIRER_a091_espMIA`, `DIRER_a092_bmsMIA`, `DIRER_a093_canHardwareBusA`, `DIRER_a094_canDataBusA`, `DIRER_a095_canOverrunBusA`, `DIRER_a096_memoryError`, `DIRER_a097_eepromError`, `DIRER_a098_intTimeTooLong`, `DIRER_a099_threadOverrun`, `DIRER_a100_assertion`, `DIRER_a101_eccTestData1`, `DIRER_a103_lowFlow`, `DIRER_a107_fpgaError`, `DIRER_a108_stateTrans`, `DIRER_a110_brakeMIA`, `DIRER_a111_badPhaseSensorCalib`, `DIRER_a112_noPhaseCurrent`, `DIRER_a113_noFuncHeatsinkSensor`, `DIRER_a116_highSpeedWearCounter`, `DIRER_a117_diMIA`, `DIRER_a119_hvpMIA`, `DIRER_a120_hvlinkMIA`, `DIRER_a121_motorControlRegulation`, `DIRER_a122_highStackUsage`, `DIRER_a123_lossMotorControl`, `DIRER_a126_limpMode`, `DIRER_a127_gracefulPowerOff`, `DIRER_a128_fpgaVersionMismatch`, `DIRER_d085_driveInverterBoardFailure`, `DIRER_d089_invalidTorqueCommand`, `DIRER_d104_resetUnintended`, `DIRER_d105_busVoltageSensorIssue`, `DIRER_d118_oilPumpCommsLost`, `DIRER_d124_oilPumpServiceRequired`, `DIRER_d125_currentSensorIssue`

### 0x3EF `DIRER_alertMatrix3` — 40 signals
`DIRER_a133_capacitorOT`, `DIRER_a134_wheelSpeedIrrational`, `DIRER_a136_spiError`, `DIRER_a142_highLashAngle`, `DIRER_a144_configMismatch`, `DIRER_a147_highTorqueWearLimit`, `DIRER_a148_burnInCycleEnded`, `DIRER_a149_oilPumpScavengeFail`, `DIRER_a150_busVD`, `DIRER_a151_shockTorqueLimiter`, `DIRER_a152_linError`, `DIRER_a153_oilPumpDiagnostics`, `DIRER_a154_resolver`, `DIRER_a155_vcfrontMIA`, `DIRER_a156_currentObserver`, `DIRER_a157_rcmMIA`, `DIRER_a158_ibstMIA`, `DIRER_a160_busVoltageAnomaly`, `DIRER_a161_epas3pMIA`, `DIRER_a162_endOfLifetimeBurnIn`, `DIRER_a172_statorOilTempOT`, `DIRER_a173_ascFdbkDiagnostic`, `DIRER_a174_unitMayNotRestart`, `DIRER_a176_safetyICFault`, `DIRER_a177_fluxReferenceCorrected`, `DIRER_d129_oilPumpIssue`, `DIRER_d130_positionSensorIssue`, `DIRER_d131_currentSensorOffset`, `DIRER_d132_busUVIssue`, `DIRER_d135_rotatedStator`, `DIRER_d137_currentSensorOutOfRange`, `DIRER_d138_inverterLVPowerSupplyLow`, `DIRER_d139_unitEEPROMIssue`, `DIRER_d141_coolingSystemIssue`, `DIRER_d143_wheelSpeedCorrelationIssue`, `DIRER_d163_recoverableOvercurrent`, `DIRER_d164_firmwareConfigMismatch`, `DIRER_d165_statorOverTemperature`, `DIRER_d166_rotorOverTemperature`, `DIRER_d167_tempSensorIssue`

### 0x3F0 `DIRER_alertMatrix4` — 20 signals
`DIRER_a202_excessHeatUnavailable`, `DIRER_a225_tmpEstPlausibility`, `DIRER_a229_busVsensorOverCAN`, `DIRER_a234_pcsMIA`, `DIRER_a236_tasMIA`, `DIRER_a240_clearFusesRoutine`, `DIRER_a241_systemThermallyLimited`, `DIRER_a242_spinDownLearning`, `DIRER_a243_ecuLogAvailable`, `DIRER_a244_mechSafeStateApplied`, `DIRER_a245_mechSafeStatePreWarn`, `DIRER_a246_recoveryAtSpeedError`, `DIRER_a247_rotorOffsetError`, `DIRER_a249_highStatorTempFromResist`, `DIRER_a250_preregulatorRail`, `DIRER_a251_oilPumpService`, `DIRER_a252_currentCoreFallback`, `DIRER_a253_lvBoostedRail`, `DIRER_a254_activeDischargeRail`, `DIRER_a255_currentGainFallback`

### 0x5AA `DIRER_alertLog` — 550 signals
Alert snapshot frame (550 signals, one group per alert). Oil-related: `DIRER_a144_oilPump1HwType`, `DIRER_a144_oilPump2HwType`, `DIRER_a251_oilFlow`, `DIRER_a251_oilTemperature`

### 0x5DB `DIRER_thermalControl` — 7 signals
`DIRER_activeInletTempReq`, `DIRER_burnInStatus`, `DIRER_coolantFlowReq`, `DIRER_criticalFlowReq`, `DIRER_oilFlowCoolingRequest`, `DIRER_oilFlowReq`, `DIRER_passiveInletTempReq`

## Rear motor (dual-motor cars) — `DIR`

### 0x108 `DIR_torque` — 9 signals
*Decoded:* `*_torqueActual` bit 27, 13 bits signed, ×2 Nm · `*_axleSpeed` bit 40, 16 bits signed, ×0.1 RPM (Model 3/Y DBC; decoded)

`DIR_axlePowerCalcQF`, `DIR_axleSpeed`, `DIR_axleSpeedQF`, `DIR_axleTorqueQF`, `DIR_gear`, `DIR_torqueActual`, `DIR_torqueChecksum`, `DIR_torqueCommand`, `DIR_torqueCounter`

### 0x256 `DIR_status` — 16 signals
`DIR_adDirection`, `DIR_adSpeedDiff`, `DIR_adState`, `DIR_axleSpeedLimitRequest`, `DIR_driveModeState`, `DIR_haltRequest`, `DIR_lvSupplyV`, `DIR_softSysLimpRequest`, `DIR_soptState`, `DIR_spinDownLearningMode`, `DIR_spinDownLearningState`, `DIR_state`, `DIR_statusChecksum`, `DIR_statusCounter`, `DIR_sysLimpRequest`, `DIR_wasteState`

### 0x279 `DIR_hvStatus` — 15 signals
`DIR_activeDischargeCtrl`, `DIR_activeDischargeStatus`, `DIR_fluxState`, `DIR_gateDriveState`, `DIR_gateDriveSupplyState`, `DIR_hvStatusChecksum`, `DIR_hvStatusCounter`, `DIR_hvilCmVoltage`, `DIR_hvilCurrent`, `DIR_hvilStatus`, `DIR_hvilSupplyState`, `DIR_powerStageSafeState`, `DIR_vBat`, `DIR_vBatQF`, `DIR_vBusDiag`

### 0x335 `DIR_info` — 25 signals
`DIR_infoIndex`, `DIR_buildType`, `DIR_buildConfigurationId`, `DIR_hardwareId`, `DIR_componentId`, `DIR_pcbaId`, `DIR_assemblyId`, `DIR_usageId`, `DIR_subUsageId`, `DIR_motorType`, `DIR_siliconType`, `DIR_mechSafeStateType`, `DIR_applicationCrc`, `DIR_oilPumpBuildType`, `DIR_oilPumpAppCrc`, `DIR_FPGA_version`, `DIR_appGitHash`, `DIR_bootGitHash`, `DIR_oilPumpPcbaId`, `DIR_oilPumpAssemblyId`, `DIR_oilPumpUsageId`, `DIR_refRotorFlux`, `DIR_refRotorOffset`, `DIR_speedWearTotal`, `DIR_torqueWearTotal`

### 0x377 `DIR_temperature` — 21 signals
`DIR_tempIndex`, `DIR_inverterTQF`, `DIR_pcbT`, `DIR_inverterT`, `DIR_statorT`, `DIR_dcCapT`, `DIR_heatsinkT`, `DIR_inverterTpct`, `DIR_statorTpct`, `DIR_fluidInTemp`, `DIR_normalFetBurnIn`, `DIR_additionalFetBurnIn`, `DIR_currentWeibullMiles`, `DIR_endOfServiceWeibullMiles`, `DIR_burnInDamageRatio`, `DIR_inverterSensorEst`, `DIR_inverterHS1Est`, `DIR_initialBurnInVehicleOdometer`, `DIR_inverterHS2Est`, `DIR_inverterInletEst`, `DIR_statorHousingTemp`

### 0x3A5 `DIR_alertMatrix1` — 57 signals
`DIR_a001_hwPhaseAgateDrive`, `DIR_a002_hwPhaseBgateDrive`, `DIR_a003_hwPhaseCgateDrive`, `DIR_a004_hwPhaseApeak`, `DIR_a005_hwPhaseBpeak`, `DIR_a006_hwPhaseCpeak`, `DIR_a007_statorAnomalyDetected`, `DIR_a008_hwEncoderA`, `DIR_a009_hwEncoderB`, `DIR_a010_unintendedReset`, `DIR_a011_swPowerStageNotReady`, `DIR_a012_hvilNotClosed`, `DIR_a013_eccError`, `DIR_a014_activeDamping`, `DIR_a015_mechSafeStateAnomaly`, `DIR_a016_safeStateApplied`, `DIR_a017_hwPedalMonitor`, `DIR_a018_hwLVSupplyUV`, `DIR_a020_hwMotorEncoder`, `DIR_a021_hwBusOV`, `DIR_a022_hw5vSupplyUV`, `DIR_a024_selfTest`, `DIR_a025_phaseApeak`, `DIR_a026_phaseBpeak`, `DIR_a027_phaseCpeak`, `DIR_a028_phaseArms`, `DIR_a029_phaseBrms`, `DIR_a030_phaseCrms`, `DIR_a031_phaseAcurrentOffset`, `DIR_a032_phaseBcurrentOffset`, `DIR_a033_phaseAcurrentSensor`, `DIR_a034_phaseBcurrentSensor`, `DIR_a035_phaseCurrentBalance`, `DIR_a036_busOV`, `DIR_a037_busUV`, `DIR_a038_busVsensor`, `DIR_a041_sdcMIA`, `DIR_a042_inletSensor`, `DIR_a043_outletOT`, `DIR_a044_outletUT`, `DIR_a045_outletSensor`, `DIR_a046_statorOT`, `DIR_a047_statorSensor1`, `DIR_a048_statorSensor2`, `DIR_a049_statorSensorDiff`, `DIR_a050_noStatorSensor`, `DIR_a053_unexpectedLatchState`, `DIR_a054_driveInverterOT`, `DIR_a055_trqCrossCheck`, `DIR_a056_ambientOT`, `DIR_a057_ambientUT`, `DIR_a058_ambientSensor`, `DIR_a059_heatsinkOT`, `DIR_a060_heatsinkUT`, `DIR_a061_heatsinkSensor`, `DIR_a062_systemLimpMode`, `DIR_a064_torqueIntervention`

### 0x3B5 `DIR_alertMatrix2` — 54 signals
`DIR_a065_canHardwareBusB`, `DIR_a066_canDataBusB`, `DIR_a067_canOverrunBusB`, `DIR_a069_rotorTempLimit`, `DIR_a070_udsTransactionInitiated`, `DIR_a071_busDisconnected`, `DIR_a072_gateDriveFaultCounter`, `DIR_a073_motorSpeed`, `DIR_a074_motorEncoder`, `DIR_a075_motorSpeedMismatch`, `DIR_a076_lvSupplyOV`, `DIR_a077_lvSupplyUV`, `DIR_a078_adcRefLow`, `DIR_a079_adcRefHigh`, `DIR_a084_eccTestData0`, `DIR_a086_activeDischargeOn`, `DIR_a087_gtwMIA`, `DIR_a088_uiMIA`, `DIR_a090_pmMIA`, `DIR_a091_espMIA`, `DIR_a092_bmsMIA`, `DIR_a093_canHardwareBusA`, `DIR_a094_canDataBusA`, `DIR_a095_canOverrunBusA`, `DIR_a096_memoryError`, `DIR_a097_eepromError`, `DIR_a098_intTimeTooLong`, `DIR_a099_threadOverrun`, `DIR_a100_assertion`, `DIR_a101_eccTestData1`, `DIR_a103_lowFlow`, `DIR_a107_fpgaError`, `DIR_a108_stateTrans`, `DIR_a110_brakeMIA`, `DIR_a111_badPhaseSensorCalib`, `DIR_a112_noPhaseCurrent`, `DIR_a113_noFuncHeatsinkSensor`, `DIR_a116_highSpeedWearCounter`, `DIR_a117_diMIA`, `DIR_a119_hvpMIA`, `DIR_a120_hvlinkMIA`, `DIR_a121_motorControlRegulation`, `DIR_a122_highStackUsage`, `DIR_a123_lossMotorControl`, `DIR_a126_limpMode`, `DIR_a127_gracefulPowerOff`, `DIR_a128_fpgaVersionMismatch`, `DIR_d085_driveInverterBoardFailure`, `DIR_d089_invalidTorqueCommand`, `DIR_d104_resetUnintended`, `DIR_d105_busVoltageSensorIssue`, `DIR_d118_oilPumpCommsLost`, `DIR_d124_oilPumpServiceRequired`, `DIR_d125_currentSensorIssue`

### 0x3C5 `DIR_alertMatrix3` — 40 signals
`DIR_a133_capacitorOT`, `DIR_a134_wheelSpeedIrrational`, `DIR_a136_spiError`, `DIR_a142_highLashAngle`, `DIR_a144_configMismatch`, `DIR_a147_highTorqueWearLimit`, `DIR_a148_burnInCycleEnded`, `DIR_a149_oilPumpFailure`, `DIR_a150_busVD`, `DIR_a151_shockTorqueLimiter`, `DIR_a152_linError`, `DIR_a153_oilPumpDiagnostics`, `DIR_a154_resolver`, `DIR_a155_vcfrontMIA`, `DIR_a156_currentObserver`, `DIR_a157_rcmMIA`, `DIR_a158_ibstMIA`, `DIR_a160_busVoltageAnomaly`, `DIR_a161_epas3pMIA`, `DIR_a162_endOfLifetimeBurnIn`, `DIR_a172_statorOilTempOT`, `DIR_a173_ascFdbkDiagnostic`, `DIR_a174_unitMayNotRestart`, `DIR_a176_safetyICFault`, `DIR_a177_fluxReferenceCorrected`, `DIR_d129_oilPumpIssue`, `DIR_d130_positionSensorIssue`, `DIR_d131_currentSensorOffset`, `DIR_d132_busUVIssue`, `DIR_d135_rotatedStator`, `DIR_d137_currentSensorOutOfRange`, `DIR_d138_inverterLVPowerSupplyLow`, `DIR_d139_unitEEPROMIssue`, `DIR_d141_coolingSystemIssue`, `DIR_d143_wheelSpeedCorrelationIssue`, `DIR_d163_recoverableOvercurrent`, `DIR_d164_firmwareConfigMismatch`, `DIR_d165_statorOverTemperature`, `DIR_d166_rotorOverTemperature`, `DIR_d167_tempSensorIssue`

### 0x3E5 `DIR_alertMatrix4` — 20 signals
`DIR_a202_excessHeatUnavailable`, `DIR_a225_tmpEstPlausibility`, `DIR_a229_busVsensorOverCAN`, `DIR_a234_pcsMIA`, `DIR_a236_tasMIA`, `DIR_a240_clearFusesRoutine`, `DIR_a241_systemThermallyLimited`, `DIR_a242_spinDownLearning`, `DIR_a243_ecuLogAvailable`, `DIR_a244_mechSafeStateApplied`, `DIR_a245_mechSafeStatePreWarn`, `DIR_a246_recoveryAtSpeedError`, `DIR_a247_rotorOffsetError`, `DIR_a249_highStatorTempFromResist`, `DIR_a250_preregulatorRail`, `DIR_a251_oilPumpService`, `DIR_a252_currentCoreFallback`, `DIR_a253_lvBoostedRail`, `DIR_a254_activeDischargeRail`, `DIR_a255_currentGainFallback`

### 0x5A5 `DIR_alertLog` — 550 signals
Alert snapshot frame (550 signals, one group per alert). Oil-related: `DIR_a144_oilPump1HwType`, `DIR_a144_oilPump2HwType`, `DIR_a251_oilFlow`, `DIR_a251_oilTemperature`

### 0x5D7 `DIR_thermalControl` — 6 signals
`DIR_activeInletTempReq`, `DIR_burnInStatus`, `DIR_coolantFlowReq`, `DIR_criticalFlowReq`, `DIR_oilFlowReq`, `DIR_passiveInletTempReq`

### 0x7EC `DIR_motorStatus` — 7 signals
*Decoded:* `*_motorCurrent` bit 11, 11 bits, ×1 A (from the Model 3/Y DBC; decoded on the Flipper Live Data screen)

`DIR_keepAliveRequest`, `DIR_motorCurrent`, `DIR_motorStatusChecksum`, `DIR_motorStatusCounter`, `DIR_switchShortTestRetryCount`, `DIR_switchingFrequency`, `DIR_targetFluxMode`
