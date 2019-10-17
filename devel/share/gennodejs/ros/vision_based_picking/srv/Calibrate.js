// Auto-generated. Do not edit!

// (in-package vision_based_picking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CalibrateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.the_request = null;
      this.cam_index = null;
    }
    else {
      if (initObj.hasOwnProperty('the_request')) {
        this.the_request = initObj.the_request
      }
      else {
        this.the_request = '';
      }
      if (initObj.hasOwnProperty('cam_index')) {
        this.cam_index = initObj.cam_index
      }
      else {
        this.cam_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrateRequest
    // Serialize message field [the_request]
    bufferOffset = _serializer.string(obj.the_request, buffer, bufferOffset);
    // Serialize message field [cam_index]
    bufferOffset = _serializer.int16(obj.cam_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrateRequest
    let len;
    let data = new CalibrateRequest(null);
    // Deserialize message field [the_request]
    data.the_request = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cam_index]
    data.cam_index = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.the_request.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vision_based_picking/CalibrateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbd3bbe48e88ac081eadeeeb8701d7f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string the_request
    int16 cam_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrateRequest(null);
    if (msg.the_request !== undefined) {
      resolved.the_request = msg.the_request;
    }
    else {
      resolved.the_request = ''
    }

    if (msg.cam_index !== undefined) {
      resolved.cam_index = msg.cam_index;
    }
    else {
      resolved.cam_index = 0
    }

    return resolved;
    }
};

class CalibrateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrateResponse
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrateResponse
    let len;
    let data = new CalibrateResponse(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vision_based_picking/CalibrateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a842b65f413084dc2b10fb484ea7f17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 z
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrateResponse(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: CalibrateRequest,
  Response: CalibrateResponse,
  md5sum() { return 'd402ff8b5a8f9b73b3f2511623d98c8d'; },
  datatype() { return 'vision_based_picking/Calibrate'; }
};
