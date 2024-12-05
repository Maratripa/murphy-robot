<template>
  <div class="min-h-screen bg-black flex items-center justify-center p-4">
    <div class="w-full max-w-lg bg-gray-900 rounded-xl shadow-2xl overflow-hidden border border-gray-800">
      <div class="p-6">
        <div class="flex items-center space-x-2 mb-2">
          <Bot class="w-8 h-8 text-white" />
          <h1 class="text-3xl font-bold text-white">Murphy Robot</h1>
        </div>
        <p class="text-gray-400 mb-6">
          Murphy es un robot capaz de segmentar heridas y colocar puntos de sutura
        </p>
        <div class="grid grid-cols-1 sm:grid-cols-2 gap-4" v-if="!isTestPosition">
          <button
            v-for="(button, index) in buttons"
            :key="index"
            @click="button.action"
            class="w-full bg-gray-800 text-white border border-gray-700 hover:bg-gray-700 hover:text-white transition-all duration-300 py-2 px-4 rounded-lg flex items-center justify-center space-x-2"
          >
            <component :is="button.icon" class="w-4 h-4" />
            <span>{{ button.name }}</span>
          </button>
        </div>
        <div class="flex flex-col gap-y-4" v-if="isTestPosition">
          <h3 class="text-white font-bold">Posición</h3>
          <div class="flex flex-col gap-y-2">
            <h2 class="text-white text-sm">Motor 1</h2>
            <input v-model="motor1Position" type="text" class="w-full p-2 bg-gray-700 text-white" />
          </div>
          <div class="flex flex-col gap-y-2">
            <h2 class="text-white text-sm">Motor 2</h2>
            <input v-model="motor2Position" type="text" class="w-full p-2 bg-gray-700 text-white" />
          </div>
          <div class="flex flex-col gap-y-2">
            <h2 class="text-white text-sm">Motor 3</h2>
            <input v-model="motor3Position" type="text" class="w-full p-2 bg-gray-700 text-white"   />
          </div>
          <div class="flex gap-x-4">
            <button @click="handleTestPosition" class="w-full bg-gray-800 text-white border border-gray-700 hover:bg-gray-700 hover:text-white transition-all duration-300 py-2 px-4 rounded-lg">
              Cancelar
            </button>
            <button @click="sendTestPosition" class="w-full bg-gray-800 text-white border border-gray-700 hover:bg-gray-700 hover:text-white transition-all duration-300 py-2 px-4 rounded-lg">
              Enviar 
            </button>
          </div>
          
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { Bot, Play, AreaChart, Repeat, Crosshair } from 'lucide-vue-next'

const result = ref(null)

const isTestPosition = ref(false)

const motor1Position = ref('')
const motor2Position = ref('')
const motor3Position = ref('')

const handleTestPosition = () => {
  isTestPosition.value = !isTestPosition.value
}

const makeRequest = async (url) => {
  const response  = await $fetch(url)

  return response
}

const sendTestPosition = async () => {
  const response = await makeRequest('/api/testPosition', {
    method: 'POST',
    body: JSON.stringify({
      motor1: parseInt(motor1Position.value),
      motor2: parseInt(motor2Position.value),
      motor3: parseInt(motor3Position.value)
    })
  })
  handleTestPosition()

  return response
}

const buttons = [
  { 
    name: "Empezar Rutina",
    icon: Play,
    action: async () => {
      console.log('Empezar Rutina')
      result.value = await makeRequest('/api/startRoutine')
    }
  },
  { 
    name: "Test Area", 
    icon: AreaChart,
    action: async () => {
      result.value = await makeRequest('/api/testArea')
    }
  },
  { 
    name: "Test Repetibilidad", 
    icon: Repeat, 
    action: async () => {
      result.value = await makeRequest('/api/testRepeatability')
    },
  },
  { 
    name: "Test Posicion", 
    icon: Crosshair, 
    action: async () => {
      handleTestPosition()
    },
  }
]

</script>