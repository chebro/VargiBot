
"use strict";

let SubmitTray = require('./SubmitTray.js')
let AGVControl = require('./AGVControl.js')
let ConveyorBeltControl = require('./ConveyorBeltControl.js')
let PopulationControl = require('./PopulationControl.js')
let VacuumGripperControl = require('./VacuumGripperControl.js')
let GetMaterialLocations = require('./GetMaterialLocations.js')

module.exports = {
  SubmitTray: SubmitTray,
  AGVControl: AGVControl,
  ConveyorBeltControl: ConveyorBeltControl,
  PopulationControl: PopulationControl,
  VacuumGripperControl: VacuumGripperControl,
  GetMaterialLocations: GetMaterialLocations,
};
