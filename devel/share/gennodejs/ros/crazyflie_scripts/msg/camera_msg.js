// Auto-generated. Do not edit!

// (in-package crazyflie_scripts.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class camera_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ext_x = null;
      this.ext_y = null;
      this.t_x = null;
      this.t_y = null;
      this.x_dir_tt = null;
      this.y_dir_tt = null;
    }
    else {
      if (initObj.hasOwnProperty('ext_x')) {
        this.ext_x = initObj.ext_x
      }
      else {
        this.ext_x = 0;
      }
      if (initObj.hasOwnProperty('ext_y')) {
        this.ext_y = initObj.ext_y
      }
      else {
        this.ext_y = 0;
      }
      if (initObj.hasOwnProperty('t_x')) {
        this.t_x = initObj.t_x
      }
      else {
        this.t_x = 0;
      }
      if (initObj.hasOwnProperty('t_y')) {
        this.t_y = initObj.t_y
      }
      else {
        this.t_y = 0;
      }
      if (initObj.hasOwnProperty('x_dir_tt')) {
        this.x_dir_tt = initObj.x_dir_tt
      }
      else {
        this.x_dir_tt = 0;
      }
      if (initObj.hasOwnProperty('y_dir_tt')) {
        this.y_dir_tt = initObj.y_dir_tt
      }
      else {
        this.y_dir_tt = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type camera_msg
    // Serialize message field [ext_x]
    bufferOffset = _serializer.int32(obj.ext_x, buffer, bufferOffset);
    // Serialize message field [ext_y]
    bufferOffset = _serializer.int32(obj.ext_y, buffer, bufferOffset);
    // Serialize message field [t_x]
    bufferOffset = _serializer.int32(obj.t_x, buffer, bufferOffset);
    // Serialize message field [t_y]
    bufferOffset = _serializer.int32(obj.t_y, buffer, bufferOffset);
    // Serialize message field [x_dir_tt]
    bufferOffset = _serializer.int32(obj.x_dir_tt, buffer, bufferOffset);
    // Serialize message field [y_dir_tt]
    bufferOffset = _serializer.int32(obj.y_dir_tt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type camera_msg
    let len;
    let data = new camera_msg(null);
    // Deserialize message field [ext_x]
    data.ext_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ext_y]
    data.ext_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [t_x]
    data.t_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [t_y]
    data.t_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_dir_tt]
    data.x_dir_tt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_dir_tt]
    data.y_dir_tt = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'crazyflie_scripts/camera_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0528b01776a9faff38d86476b22e6f80';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 ext_x
    int32 ext_y
    int32 t_x
    int32 t_y
    int32 x_dir_tt
    int32 y_dir_tt
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new camera_msg(null);
    if (msg.ext_x !== undefined) {
      resolved.ext_x = msg.ext_x;
    }
    else {
      resolved.ext_x = 0
    }

    if (msg.ext_y !== undefined) {
      resolved.ext_y = msg.ext_y;
    }
    else {
      resolved.ext_y = 0
    }

    if (msg.t_x !== undefined) {
      resolved.t_x = msg.t_x;
    }
    else {
      resolved.t_x = 0
    }

    if (msg.t_y !== undefined) {
      resolved.t_y = msg.t_y;
    }
    else {
      resolved.t_y = 0
    }

    if (msg.x_dir_tt !== undefined) {
      resolved.x_dir_tt = msg.x_dir_tt;
    }
    else {
      resolved.x_dir_tt = 0
    }

    if (msg.y_dir_tt !== undefined) {
      resolved.y_dir_tt = msg.y_dir_tt;
    }
    else {
      resolved.y_dir_tt = 0
    }

    return resolved;
    }
};

module.exports = camera_msg;
