
"use strict";

let IsProgramSaved = require('./IsProgramSaved.js')
let AddToLog = require('./AddToLog.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let Popup = require('./Popup.js')
let GetProgramState = require('./GetProgramState.js')
let RawRequest = require('./RawRequest.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')
let GetSafetyMode = require('./GetSafetyMode.js')

module.exports = {
  IsProgramSaved: IsProgramSaved,
  AddToLog: AddToLog,
  GetRobotMode: GetRobotMode,
  GetLoadedProgram: GetLoadedProgram,
  Popup: Popup,
  GetProgramState: GetProgramState,
  RawRequest: RawRequest,
  IsProgramRunning: IsProgramRunning,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
  GetSafetyMode: GetSafetyMode,
};
