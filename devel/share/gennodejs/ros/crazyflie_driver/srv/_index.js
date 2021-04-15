
"use strict";

let AddCrazyflie = require('./AddCrazyflie.js')
let RemoveCrazyflie = require('./RemoveCrazyflie.js')
let UploadTrajectory = require('./UploadTrajectory.js')
let GoTo = require('./GoTo.js')
let Stop = require('./Stop.js')
let Land = require('./Land.js')
let Takeoff = require('./Takeoff.js')
let sendPacket = require('./sendPacket.js')
let UpdateParams = require('./UpdateParams.js')
let SetGroupMask = require('./SetGroupMask.js')
let StartTrajectory = require('./StartTrajectory.js')

module.exports = {
  AddCrazyflie: AddCrazyflie,
  RemoveCrazyflie: RemoveCrazyflie,
  UploadTrajectory: UploadTrajectory,
  GoTo: GoTo,
  Stop: Stop,
  Land: Land,
  Takeoff: Takeoff,
  sendPacket: sendPacket,
  UpdateParams: UpdateParams,
  SetGroupMask: SetGroupMask,
  StartTrajectory: StartTrajectory,
};
