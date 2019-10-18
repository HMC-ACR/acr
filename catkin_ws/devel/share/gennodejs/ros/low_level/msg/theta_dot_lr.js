// Auto-generated. Do not edit!

// (in-package low_level.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class theta_dot_lr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_dot_left = null;
      this.theta_dot_right = null;
    }
    else {
      if (initObj.hasOwnProperty('theta_dot_left')) {
        this.theta_dot_left = initObj.theta_dot_left
      }
      else {
        this.theta_dot_left = 0.0;
      }
      if (initObj.hasOwnProperty('theta_dot_right')) {
        this.theta_dot_right = initObj.theta_dot_right
      }
      else {
        this.theta_dot_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type theta_dot_lr
    // Serialize message field [theta_dot_left]
    bufferOffset = _serializer.float32(obj.theta_dot_left, buffer, bufferOffset);
    // Serialize message field [theta_dot_right]
    bufferOffset = _serializer.float32(obj.theta_dot_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type theta_dot_lr
    let len;
    let data = new theta_dot_lr(null);
    // Deserialize message field [theta_dot_left]
    data.theta_dot_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_dot_right]
    data.theta_dot_right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'low_level/theta_dot_lr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd5df01c85a05c89a29d456cc9a16601';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 theta_dot_left
    float32 theta_dot_right
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new theta_dot_lr(null);
    if (msg.theta_dot_left !== undefined) {
      resolved.theta_dot_left = msg.theta_dot_left;
    }
    else {
      resolved.theta_dot_left = 0.0
    }

    if (msg.theta_dot_right !== undefined) {
      resolved.theta_dot_right = msg.theta_dot_right;
    }
    else {
      resolved.theta_dot_right = 0.0
    }

    return resolved;
    }
};

module.exports = theta_dot_lr;
