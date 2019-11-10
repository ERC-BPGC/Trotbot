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
        this.path = new std_msgs.msg.Float32MultiArray();
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
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.path, buffer, bufferOffset);
    // Serialize message field [ack]
    bufferOffset = _serializer.bool(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerResponse
    let len;
    let data = new PlannerResponse(null);
    // Deserialize message field [path]
    data.path = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [ack]
    data.ack = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.path);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'navigation/PlannerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b62fc16eef217a484b6eb0641685257';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32MultiArray path
    bool ack
    
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerResponse(null);
    if (msg.path !== undefined) {
      resolved.path = std_msgs.msg.Float32MultiArray.Resolve(msg.path)
    }
    else {
      resolved.path = new std_msgs.msg.Float32MultiArray()
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
  md5sum() { return '59d496e471e2ce1a2e0dd836fb1cdfb4'; },
  datatype() { return 'navigation/Planner'; }
};
