// Auto-generated. Do not edit!

// (in-package car.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class pid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.setpoint = null;
      this.laster = null;
      this.err = null;
      this.wndup = null;
      this.kp = null;
      this.kd = null;
      this.ki = null;
      this.out = null;
      this.fdbck = null;
      this.iterm = null;
      this.pterm = null;
      this.dterm = null;
      this.delterr = null;
      this.delttime = null;
    }
    else {
      if (initObj.hasOwnProperty('setpoint')) {
        this.setpoint = initObj.setpoint
      }
      else {
        this.setpoint = 0.0;
      }
      if (initObj.hasOwnProperty('laster')) {
        this.laster = initObj.laster
      }
      else {
        this.laster = 0.0;
      }
      if (initObj.hasOwnProperty('err')) {
        this.err = initObj.err
      }
      else {
        this.err = 0.0;
      }
      if (initObj.hasOwnProperty('wndup')) {
        this.wndup = initObj.wndup
      }
      else {
        this.wndup = 0.0;
      }
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = 0.0;
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = 0.0;
      }
      if (initObj.hasOwnProperty('ki')) {
        this.ki = initObj.ki
      }
      else {
        this.ki = 0.0;
      }
      if (initObj.hasOwnProperty('out')) {
        this.out = initObj.out
      }
      else {
        this.out = 0.0;
      }
      if (initObj.hasOwnProperty('fdbck')) {
        this.fdbck = initObj.fdbck
      }
      else {
        this.fdbck = 0.0;
      }
      if (initObj.hasOwnProperty('iterm')) {
        this.iterm = initObj.iterm
      }
      else {
        this.iterm = 0.0;
      }
      if (initObj.hasOwnProperty('pterm')) {
        this.pterm = initObj.pterm
      }
      else {
        this.pterm = 0.0;
      }
      if (initObj.hasOwnProperty('dterm')) {
        this.dterm = initObj.dterm
      }
      else {
        this.dterm = 0.0;
      }
      if (initObj.hasOwnProperty('delterr')) {
        this.delterr = initObj.delterr
      }
      else {
        this.delterr = 0.0;
      }
      if (initObj.hasOwnProperty('delttime')) {
        this.delttime = initObj.delttime
      }
      else {
        this.delttime = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pid
    // Serialize message field [setpoint]
    bufferOffset = _serializer.float32(obj.setpoint, buffer, bufferOffset);
    // Serialize message field [laster]
    bufferOffset = _serializer.float32(obj.laster, buffer, bufferOffset);
    // Serialize message field [err]
    bufferOffset = _serializer.float32(obj.err, buffer, bufferOffset);
    // Serialize message field [wndup]
    bufferOffset = _serializer.float32(obj.wndup, buffer, bufferOffset);
    // Serialize message field [kp]
    bufferOffset = _serializer.float32(obj.kp, buffer, bufferOffset);
    // Serialize message field [kd]
    bufferOffset = _serializer.float32(obj.kd, buffer, bufferOffset);
    // Serialize message field [ki]
    bufferOffset = _serializer.float32(obj.ki, buffer, bufferOffset);
    // Serialize message field [out]
    bufferOffset = _serializer.float32(obj.out, buffer, bufferOffset);
    // Serialize message field [fdbck]
    bufferOffset = _serializer.float64(obj.fdbck, buffer, bufferOffset);
    // Serialize message field [iterm]
    bufferOffset = _serializer.float32(obj.iterm, buffer, bufferOffset);
    // Serialize message field [pterm]
    bufferOffset = _serializer.float32(obj.pterm, buffer, bufferOffset);
    // Serialize message field [dterm]
    bufferOffset = _serializer.float32(obj.dterm, buffer, bufferOffset);
    // Serialize message field [delterr]
    bufferOffset = _serializer.float32(obj.delterr, buffer, bufferOffset);
    // Serialize message field [delttime]
    bufferOffset = _serializer.float32(obj.delttime, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pid
    let len;
    let data = new pid(null);
    // Deserialize message field [setpoint]
    data.setpoint = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [laster]
    data.laster = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [err]
    data.err = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wndup]
    data.wndup = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kp]
    data.kp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kd]
    data.kd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ki]
    data.ki = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [out]
    data.out = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fdbck]
    data.fdbck = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [iterm]
    data.iterm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pterm]
    data.pterm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dterm]
    data.dterm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delterr]
    data.delterr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delttime]
    data.delttime = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'car/pid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87da0e9e14c5b44deff5947cc352d507';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 setpoint
    float32 laster
    float32 err
    float32 wndup
    float32 kp
    float32 kd
    float32 ki
    float32 out
    float64 fdbck
    float32 iterm
    float32 pterm
    float32 dterm
    float32 delterr
    float32 delttime
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pid(null);
    if (msg.setpoint !== undefined) {
      resolved.setpoint = msg.setpoint;
    }
    else {
      resolved.setpoint = 0.0
    }

    if (msg.laster !== undefined) {
      resolved.laster = msg.laster;
    }
    else {
      resolved.laster = 0.0
    }

    if (msg.err !== undefined) {
      resolved.err = msg.err;
    }
    else {
      resolved.err = 0.0
    }

    if (msg.wndup !== undefined) {
      resolved.wndup = msg.wndup;
    }
    else {
      resolved.wndup = 0.0
    }

    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = 0.0
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = 0.0
    }

    if (msg.ki !== undefined) {
      resolved.ki = msg.ki;
    }
    else {
      resolved.ki = 0.0
    }

    if (msg.out !== undefined) {
      resolved.out = msg.out;
    }
    else {
      resolved.out = 0.0
    }

    if (msg.fdbck !== undefined) {
      resolved.fdbck = msg.fdbck;
    }
    else {
      resolved.fdbck = 0.0
    }

    if (msg.iterm !== undefined) {
      resolved.iterm = msg.iterm;
    }
    else {
      resolved.iterm = 0.0
    }

    if (msg.pterm !== undefined) {
      resolved.pterm = msg.pterm;
    }
    else {
      resolved.pterm = 0.0
    }

    if (msg.dterm !== undefined) {
      resolved.dterm = msg.dterm;
    }
    else {
      resolved.dterm = 0.0
    }

    if (msg.delterr !== undefined) {
      resolved.delterr = msg.delterr;
    }
    else {
      resolved.delterr = 0.0
    }

    if (msg.delttime !== undefined) {
      resolved.delttime = msg.delttime;
    }
    else {
      resolved.delttime = 0.0
    }

    return resolved;
    }
};

module.exports = pid;
