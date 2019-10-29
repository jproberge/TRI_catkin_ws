
"use strict";

let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let Analog = require('./Analog.js');
let IOStates = require('./IOStates.js');
let Digital = require('./Digital.js');

module.exports = {
  RobotModeDataMsg: RobotModeDataMsg,
  ToolDataMsg: ToolDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  MasterboardDataMsg: MasterboardDataMsg,
  Analog: Analog,
  IOStates: IOStates,
  Digital: Digital,
};
