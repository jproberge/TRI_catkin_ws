// Auto-generated. Do not edit!

// (in-package ur_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripperMoveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.force = null;
      this.speed = null;
      this.position_compensation = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = 0;
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = 0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('position_compensation')) {
        this.position_compensation = initObj.position_compensation
      }
      else {
        this.position_compensation = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperMoveRequest
    // Serialize message field [pos]
    bufferOffset = _serializer.uint8(obj.pos, buffer, bufferOffset);
    // Serialize message field [force]
    bufferOffset = _serializer.uint8(obj.force, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.uint8(obj.speed, buffer, bufferOffset);
    // Serialize message field [position_compensation]
    bufferOffset = _serializer.bool(obj.position_compensation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperMoveRequest
    let len;
    let data = new GripperMoveRequest(null);
    // Deserialize message field [pos]
    data.pos = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [force]
    data.force = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position_compensation]
    data.position_compensation = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/GripperMoveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e1410c2dff620c967ae982f954af617';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 pos
    uint8 force
    uint8 speed
    bool position_compensation
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperMoveRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = 0
    }

    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = 0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.position_compensation !== undefined) {
      resolved.position_compensation = msg.position_compensation;
    }
    else {
      resolved.position_compensation = false
    }

    return resolved;
    }
};

class GripperMoveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.has_object = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('has_object')) {
        this.has_object = initObj.has_object
      }
      else {
        this.has_object = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperMoveResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [has_object]
    bufferOffset = _serializer.bool(obj.has_object, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperMoveResponse
    let len;
    let data = new GripperMoveResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_object]
    data.has_object = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/GripperMoveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7c3e9566a7011a43c48ce986ef29228';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    bool has_object
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperMoveResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.has_object !== undefined) {
      resolved.has_object = msg.has_object;
    }
    else {
      resolved.has_object = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GripperMoveRequest,
  Response: GripperMoveResponse,
  md5sum() { return 'd1d55f75257890b063dd76689fbde353'; },
  datatype() { return 'ur_msgs/GripperMove'; }
};
