// Auto-generated. Do not edit!

// (in-package navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point_xy = require('../msg/Point_xy.js');
let PolyArray = require('../msg/PolyArray.js');

//-----------------------------------------------------------

let PointArray = require('../msg/PointArray.js');

//-----------------------------------------------------------

class PlannerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.goal = null;
      this.obstacle_list = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = new Point_xy();
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new Point_xy();
      }
      if (initObj.hasOwnProperty('obstacle_list')) {
        this.obstacle_list = initObj.obstacle_list
      }
      else {
        this.obstacle_list = new PolyArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerRequest
    // Serialize message field [start]
    bufferOffset = Point_xy.serialize(obj.start, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = Point_xy.serialize(obj.goal, buffer, bufferOffset);
    // Serialize message field [obstacle_list]
    bufferOffset = PolyArray.serialize(obj.obstacle_list, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerRequest
    let len;
    let data = new PlannerRequest(null);
    // Deserialize message field [start]
    data.start = Point_xy.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = Point_xy.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle_list]
    data.obstacle_list = PolyArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Point_xy.getMessageSize(object.start);
    length += Point_xy.getMessageSize(object.goal);
    length += PolyArray.getMessageSize(object.obstacle_list);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/PlannerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '312cafca219eff06ca8155401fd152ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    navigation/Point_xy start
    navigation/Point_xy goal
    navigation/PolyArray obstacle_list
    
    ================================================================================
    MSG: navigation/Point_xy
    float32[] point
    ================================================================================
    MSG: navigation/PolyArray
    navigation/PointArray[] polygons
    
    ================================================================================
    MSG: navigation/PointArray
    navigation/Point_xy[] points
      
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerRequest(null);
    if (msg.start !== undefined) {
      resolved.start = Point_xy.Resolve(msg.start)
    }
    else {
      resolved.start = new Point_xy()
    }

    if (msg.goal !== undefined) {
      resolved.goal = Point_xy.Resolve(msg.goal)
    }
    else {
      resolved.goal = new Point_xy()
    }

    if (msg.obstacle_list !== undefined) {
      resolved.obstacle_list = PolyArray.Resolve(msg.obstacle_list)
    }
    else {
      resolved.obstacle_list = new PolyArray()
    }

    return resolved;
    }
};

class PlannerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
      this.ack = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new PointArray();
      }
      if (initObj.hasOwnProperty('ack')) {
        this.ack = initObj.ack
      }
      else {
        this.ack = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerResponse
    // Serialize message field [path]
    bufferOffset = PointArray.serialize(obj.path, buffer, bufferOffset);
    // Serialize message field [ack]
    bufferOffset = _serializer.bool(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerResponse
    let len;
    let data = new PlannerResponse(null);
    // Deserialize message field [path]
    data.path = PointArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [ack]
    data.ack = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PointArray.getMessageSize(object.path);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/PlannerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2ffe07356360c7bae31566a65032850';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    navigation/PointArray path
    bool ack
    
    ================================================================================
    MSG: navigation/PointArray
    navigation/Point_xy[] points
      
    ================================================================================
    MSG: navigation/Point_xy
    float32[] point
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerResponse(null);
    if (msg.path !== undefined) {
      resolved.path = PointArray.Resolve(msg.path)
    }
    else {
      resolved.path = new PointArray()
    }

    if (msg.ack !== undefined) {
      resolved.ack = msg.ack;
    }
    else {
      resolved.ack = false
    }

    return resolved;
    }
};

module.exports = {
  Request: PlannerRequest,
  Response: PlannerResponse,
  md5sum() { return '0f8ba09d5a21e9916f0e3bf633247872'; },
  datatype() { return 'navigation/Planner'; }
};
