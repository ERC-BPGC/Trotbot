// Auto-generated. Do not edit!

// (in-package navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Point_xy {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Point_xy
    // Serialize message field [point]
    bufferOffset = _arraySerializer.float32(obj.point, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Point_xy
    let len;
    let data = new Point_xy(null);
    // Deserialize message field [point]
    data.point = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.point.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation/Point_xy';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '318ea976b093c91a3f95a8e83351f8ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] point
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Point_xy(null);
    if (msg.point !== undefined) {
      resolved.point = msg.point;
    }
    else {
      resolved.point = []
    }

    return resolved;
    }
};

module.exports = Point_xy;
