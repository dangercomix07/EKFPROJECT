// Auto-generated. Do not edit!

// (in-package course_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Landmark = require('./Landmark.js');

//-----------------------------------------------------------

class Trilateration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.landmarkA = null;
      this.landmarkB = null;
      this.landmarkC = null;
    }
    else {
      if (initObj.hasOwnProperty('landmarkA')) {
        this.landmarkA = initObj.landmarkA
      }
      else {
        this.landmarkA = new Landmark();
      }
      if (initObj.hasOwnProperty('landmarkB')) {
        this.landmarkB = initObj.landmarkB
      }
      else {
        this.landmarkB = new Landmark();
      }
      if (initObj.hasOwnProperty('landmarkC')) {
        this.landmarkC = initObj.landmarkC
      }
      else {
        this.landmarkC = new Landmark();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trilateration
    // Serialize message field [landmarkA]
    bufferOffset = Landmark.serialize(obj.landmarkA, buffer, bufferOffset);
    // Serialize message field [landmarkB]
    bufferOffset = Landmark.serialize(obj.landmarkB, buffer, bufferOffset);
    // Serialize message field [landmarkC]
    bufferOffset = Landmark.serialize(obj.landmarkC, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trilateration
    let len;
    let data = new Trilateration(null);
    // Deserialize message field [landmarkA]
    data.landmarkA = Landmark.deserialize(buffer, bufferOffset);
    // Deserialize message field [landmarkB]
    data.landmarkB = Landmark.deserialize(buffer, bufferOffset);
    // Deserialize message field [landmarkC]
    data.landmarkC = Landmark.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'course_project/Trilateration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45e1ed04607c6f7e36ae2697ced8826f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Landmark landmarkA
    Landmark landmarkB
    Landmark landmarkC
    
    ================================================================================
    MSG: course_project/Landmark
    float32 x
    float32 y
    float32 distance
    float32 variance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trilateration(null);
    if (msg.landmarkA !== undefined) {
      resolved.landmarkA = Landmark.Resolve(msg.landmarkA)
    }
    else {
      resolved.landmarkA = new Landmark()
    }

    if (msg.landmarkB !== undefined) {
      resolved.landmarkB = Landmark.Resolve(msg.landmarkB)
    }
    else {
      resolved.landmarkB = new Landmark()
    }

    if (msg.landmarkC !== undefined) {
      resolved.landmarkC = Landmark.Resolve(msg.landmarkC)
    }
    else {
      resolved.landmarkC = new Landmark()
    }

    return resolved;
    }
};

module.exports = Trilateration;
