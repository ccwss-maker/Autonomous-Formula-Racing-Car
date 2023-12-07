// Auto-generated. Do not edit!

// (in-package kbd_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class kbd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.w = null;
      this.a = null;
      this.s = null;
      this.d = null;
    }
    else {
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = 0;
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0;
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = 0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type kbd
    // Serialize message field [w]
    bufferOffset = _serializer.int16(obj.w, buffer, bufferOffset);
    // Serialize message field [a]
    bufferOffset = _serializer.int16(obj.a, buffer, bufferOffset);
    // Serialize message field [s]
    bufferOffset = _serializer.int16(obj.s, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.int16(obj.d, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type kbd
    let len;
    let data = new kbd(null);
    // Deserialize message field [w]
    data.w = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a]
    data.a = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [s]
    data.s = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kbd_ros_msgs/kbd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a1716104afcb5f80164140a20101628';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 w
    int16 a
    int16 s
    int16 d
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new kbd(null);
    if (msg.w !== undefined) {
      resolved.w = msg.w;
    }
    else {
      resolved.w = 0
    }

    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = 0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0
    }

    return resolved;
    }
};

module.exports = kbd;
