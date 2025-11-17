// src/RosConnector.js
import ROSLIB from 'roslib'

export default class RosConnector {
  constructor(rosbridgeUrl = 'ws://localhost:9090') {
    this.ros = new ROSLIB.Ros({
      url: rosbridgeUrl,
    })

    this.ros.on('connection', () => {
      console.log('✅ Connected to rosbridge.')
    })

    this.ros.on('error', (error) => {
      console.error('❌ Error connecting to rosbridge:', error)
    })

    this.ros.on('close', () => {
      console.log('🔌 Disconnected from rosbridge.')
    })
  }

  subscribeToVehicleStatus(callback) {
    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: '/vehicle/status',
      messageType: 'std_msgs/String',
    })

    listener.subscribe((message) => {
      try {
        const data = JSON.parse(message.data)
        callback(data)
      } catch (e) {
        console.warn('無法解析收到的 JSON:', message.data)
      }
    })

    return listener
  }
}
