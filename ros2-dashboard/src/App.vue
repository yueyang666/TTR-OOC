<template>
  <div class="container">
    <h1>🚗 車輛參數即時監控儀表板</h1>

    <div v-if="statusData" class="card">
      <div v-for="(item, key) in statusData" :key="key" class="item">

        <strong>{{ key }}:</strong>

        <span v-if="item && item.raw !== undefined">
          {{ item.raw }} {{ item.unit }}
        </span>

        <span v-else class="null">
          無資料
        </span>

      </div>
    </div>

    <div v-else class="waiting">
      等待 ROS2 資料中...
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from 'vue'
import RosConnector from './RosConnector'

const statusData = ref(null)

onMounted(() => {
  const ros = new RosConnector('ws://192.168.0.107:9090')
  ros.subscribeToVehicleStatus((data) => {
    statusData.value = data
  })
})
</script>

<style scoped>
.container {
  max-width: 600px;
  margin: auto;
  padding: 2rem;
  font-family: 'Arial', sans-serif;
}
.card {
  background: #f7f7f7;
  padding: 1.5rem;
  border-radius: 10px;
  box-shadow: 0 0 10px #ccc;
}
.item {
  margin: 0.5rem 0;
  font-size: 1.2rem;
}
.waiting {
  color: gray;
  font-style: italic;
}
</style>
