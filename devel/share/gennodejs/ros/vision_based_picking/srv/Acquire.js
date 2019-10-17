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

class AcquireRequest {
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
    // Serializes a message object of type AcquireRequest
    // Serialize message field [the_request]
    bufferOffset = _serializer.string(obj.the_request, buffer, bufferOffset);
    // Serialize message field [cam_index]
    bufferOffset = _serializer.int16(obj.cam_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AcquireRequest
    let len;
    let data = new AcquireRequest(null);
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
    return 'vision_based_picking/AcquireRequest';
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
    const resolved = new AcquireRequest(null);
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

class AcquireResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AcquireResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int16(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AcquireResponse
    let len;
    let data = new AcquireResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vision_based_picking/AcquireResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46fcc1fee3807f5925730339e5177777';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AcquireResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: AcquireRequest,
  Response: AcquireResponse,
  md5sum() { return '4cd58e4f9987399415b3ab1f2afe4e12'; },
  datatype() { return 'vision_based_picking/Acquire'; }
};
