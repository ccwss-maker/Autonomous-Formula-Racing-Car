// Auto-generated. Do not edit!

// (in-package box_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Image {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.height = null;
      this.width = null;
      this.encoding = null;
      this.is_bigendian = null;
      this.step = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('encoding')) {
        this.encoding = initObj.encoding
      }
      else {
        this.encoding = '';
      }
      if (initObj.hasOwnProperty('is_bigendian')) {
        this.is_bigendian = initObj.is_bigendian
      }
      else {
        this.is_bigendian = 0;
      }
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Image
    // Serialize message field [height]
    bufferOffset = _serializer.uint32(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [encoding]
    bufferOffset = _serializer.string(obj.encoding, buffer, bufferOffset);
    // Serialize message field [is_bigendian]
    bufferOffset = _serializer.uint8(obj.is_bigendian, buffer, bufferOffset);
    // Serialize message field [step]
    bufferOffset = _serializer.uint32(obj.step, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.uint8(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Image
    let len;
    let data = new Image(null);
    // Deserialize message field [height]
    data.height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [encoding]
    data.encoding = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_bigendian]
    data.is_bigendian = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [step]
    data.step = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.encoding);
    length += object.data.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'box_ros_msgs/Image';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '585ac495236028e67bb0be633f129591';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 height
    uint32 width
    string encoding
    uint8 is_bigendian
    uint32 step
    uint8[] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Image(null);
    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.encoding !== undefined) {
      resolved.encoding = msg.encoding;
    }
    else {
      resolved.encoding = ''
    }

    if (msg.is_bigendian !== undefined) {
      resolved.is_bigendian = msg.is_bigendian;
    }
    else {
      resolved.is_bigendian = 0
    }

    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = Image;
