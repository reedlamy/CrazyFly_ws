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
      this.ext_x_cam = null;
      this.ext_y_cam = null;
      this.track_x = null;
      this.track_y = null;
      this.track_z = null;
      this.x_dir_tt = null;
      this.y_dir_tt = null;
      this.tg_yaw_t = null;
    }
    else {
      if (initObj.hasOwnProperty('ext_x_cam')) {
        this.ext_x_cam = initObj.ext_x_cam
      }
      else {
        this.ext_x_cam = 0.0;
      }
      if (initObj.hasOwnProperty('ext_y_cam')) {
        this.ext_y_cam = initObj.ext_y_cam
      }
      else {
        this.ext_y_cam = 0.0;
      }
      if (initObj.hasOwnProperty('track_x')) {
        this.track_x = initObj.track_x
      }
      else {
        this.track_x = 0.0;
      }
      if (initObj.hasOwnProperty('track_y')) {
        this.track_y = initObj.track_y
      }
      else {
        this.track_y = 0.0;
      }
      if (initObj.hasOwnProperty('track_z')) {
        this.track_z = initObj.track_z
      }
      else {
        this.track_z = 0.0;
      }
      if (initObj.hasOwnProperty('x_dir_tt')) {
        this.x_dir_tt = initObj.x_dir_tt
      }
      else {
        this.x_dir_tt = 0.0;
      }
      if (initObj.hasOwnProperty('y_dir_tt')) {
        this.y_dir_tt = initObj.y_dir_tt
      }
      else {
        this.y_dir_tt = 0.0;
      }
      if (initObj.hasOwnProperty('tg_yaw_t')) {
        this.tg_yaw_t = initObj.tg_yaw_t
      }
      else {
        this.tg_yaw_t = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type camera_msg
    // Serialize message field [ext_x_cam]
    bufferOffset = _serializer.float32(obj.ext_x_cam, buffer, bufferOffset);
    // Serialize message field [ext_y_cam]
    bufferOffset = _serializer.float32(obj.ext_y_cam, buffer, bufferOffset);
    // Serialize message field [track_x]
    bufferOffset = _serializer.float32(obj.track_x, buffer, bufferOffset);
    // Serialize message field [track_y]
    bufferOffset = _serializer.float32(obj.track_y, buffer, bufferOffset);
    // Serialize message field [track_z]
    bufferOffset = _serializer.float32(obj.track_z, buffer, bufferOffset);
    // Serialize message field [x_dir_tt]
    bufferOffset = _serializer.float32(obj.x_dir_tt, buffer, bufferOffset);
    // Serialize message field [y_dir_tt]
    bufferOffset = _serializer.float32(obj.y_dir_tt, buffer, bufferOffset);
    // Serialize message field [tg_yaw_t]
    bufferOffset = _serializer.float32(obj.tg_yaw_t, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type camera_msg
    let len;
    let data = new camera_msg(null);
    // Deserialize message field [ext_x_cam]
    data.ext_x_cam = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ext_y_cam]
    data.ext_y_cam = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_x]
    data.track_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_y]
    data.track_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track_z]
    data.track_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_dir_tt]
    data.x_dir_tt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_dir_tt]
    data.y_dir_tt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tg_yaw_t]
    data.tg_yaw_t = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'crazyflie_scripts/camera_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '512cd6d5f46f8407166dee36dcd1fc07';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 ext_x_cam
    float32 ext_y_cam
    float32 track_x
    float32 track_y
    float32 track_z
    float32 x_dir_tt
    float32 y_dir_tt
    float32 tg_yaw_t
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new camera_msg(null);
    if (msg.ext_x_cam !== undefined) {
      resolved.ext_x_cam = msg.ext_x_cam;
    }
    else {
      resolved.ext_x_cam = 0.0
    }

    if (msg.ext_y_cam !== undefined) {
      resolved.ext_y_cam = msg.ext_y_cam;
    }
    else {
      resolved.ext_y_cam = 0.0
    }

    if (msg.track_x !== undefined) {
      resolved.track_x = msg.track_x;
    }
    else {
      resolved.track_x = 0.0
    }

    if (msg.track_y !== undefined) {
      resolved.track_y = msg.track_y;
    }
    else {
      resolved.track_y = 0.0
    }

    if (msg.track_z !== undefined) {
      resolved.track_z = msg.track_z;
    }
    else {
      resolved.track_z = 0.0
    }

    if (msg.x_dir_tt !== undefined) {
      resolved.x_dir_tt = msg.x_dir_tt;
    }
    else {
      resolved.x_dir_tt = 0.0
    }

    if (msg.y_dir_tt !== undefined) {
      resolved.y_dir_tt = msg.y_dir_tt;
    }
    else {
      resolved.y_dir_tt = 0.0
    }

    if (msg.tg_yaw_t !== undefined) {
      resolved.tg_yaw_t = msg.tg_yaw_t;
    }
    else {
      resolved.tg_yaw_t = 0.0
    }

    return resolved;
    }
};

module.exports = camera_msg;
