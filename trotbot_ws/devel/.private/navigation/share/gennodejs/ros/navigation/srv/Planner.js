// Auto-generated. Do not edit!

// (in-package navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

let PointArray = require('../msg/PointArray.js');

//-----------------------------------------------------------

class PlannerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.goal = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = [];
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerRequest
    // Serialize message field [start]
    // Serialize the length for message field [start]
    bufferOffset = _serializer.uint32(obj.start.length, buffer, bufferOffset);
    obj.start.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [goal]
    // Serialize the length for message field [goal]
    bufferOffset = _serializer.uint32(obj.goal.length, buffer, bufferOffset);
    obj.goal.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerRequest
    let len;
    let data = new PlannerRequest(null);
    // Deserialize message field [start]
    // Deserialize array length for message field [start]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.start = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.start[i] = std_msgs.msg.Float32.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [goal]
    // Deserialize array length for message field [goal]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.goal = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.goal[i] = std_msgs.msg.Float32.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.start.length;
    length += 4 * object.goal.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/PlannerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd1a8738d5147232a034239634382b47';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32[] start
    std_msgs/Float32[] goal
    
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerRequest(null);
    if (msg.start !== undefined) {
      resolved.start = new Array(msg.start.length);
      for (let i = 0; i < resolved.start.length; ++i) {
        resolved.start[i] = std_msgs.msg.Float32.Resolve(msg.start[i]);
      }
    }
    else {
      resolved.start = []
    }

    if (msg.goal !== undefined) {
      resolved.goal = new Array(msg.goal.length);
      for (let i = 0; i < resolved.goal.length; ++i) {
        resolved.goal[i] = std_msgs.msg.Float32.Resolve(msg.goal[i]);
      }
    }
    else {
      resolved.goal = []
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
    return '7dbb4339dd73402726335b4f58e3f859';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    navigation/PointArray path
    bool ack
    
    ================================================================================
    MSG: navigation/PointArray
    std_msgs/Header header
    geometry_msgs/Point[] points
      
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
  md5sum() { return 'e099e7e7ce9aa3ba23a5f56a66d6f6e9'; },
  datatype() { return 'navigation/Planner'; }
};
