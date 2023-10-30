<template>
  <div class="app">
    <!-- Control Panel on the left -->
    <div class="control-panel">
      <button @click="toggleCam('cam1', B_URL1)">Cam1</button>
      <button @click="toggleCam('cam2', B_URL2)">Cam2</button>
      <button @click="toggleCam('cam7', B_URL3)">Cam3</button>
      <button @click="toggleCam('cam4', B_URL4)">Cam4</button>
    </div>

    <!-- Video Feeds on the right -->
    <div class="video-feeds">
      <!-- Cameras -->
      <div class="camera-row">
        <div class="camera-container" v-if="cam1">
          <img :src="B_URL1 + '/video_feed/cam1'" />
        </div>
        <div class="camera-placeholder" v-else>
          Cam1
        </div>

        <div class="camera-container" v-if="cam2">
          <img :src="B_URL2 + '/video_feed/cam2'" />
        </div>
        <div class="camera-placeholder" v-else>
          Cam2
        </div>

        <div class="camera-container" v-if="cam7">
          <img :src="B_URL3 + '/video_feed/cam7'" />
        </div>
        <div class="camera-placeholder" v-else>
          Cam3
        </div>

        <div class="camera-container" v-if="cam4">
          <img :src="B_URL4 + '/video_feed/cam4'" />
        </div>
        <div class="camera-placeholder" v-else>
          Cam4
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: "App",
  data() {
    return {
      B_URL1: `${window.location.protocol}//${window.location.hostname}:5000`,
      B_URL2: `${window.location.protocol}//${window.location.hostname}:5001`,
      B_URL3: `${window.location.protocol}//${window.location.hostname}:5002`,
      B_URL4: `${window.location.protocol}//${window.location.hostname}:5003`,
      cam1: false,
      cam2: false,
      cam7: false,
      cam4: false
    };
  },
  mounted() {
    this.socket = new WebSocket(`ws://${window.location.hostname}`);
    this.socket.onmessage = (event) => {
      console.log("Got ws msg "+JSON.stringify(event));
    };
  },
  methods: {
    toggleCam(cam_id, url) {
      if (this[cam_id]) {
        fetch(`${url}/turn_off/${cam_id}`, { method: 'POST' })
          .then(response => response.json())
          .then(data => {
            if (data.success) {
              this[cam_id] = false;
            }
          });
      } else {
        fetch(`${url}/turn_on/${cam_id}`, { method: 'POST' })
          .then(response => response.json())
          .then(data => {
            if (data.success) {
              this[cam_id] = true;
            }
          });
      }
    }
  }
};
</script>

<style scoped>
.control-panel {
  width: 60px;
  float: left;
  height: 100vh;
  background: grey;
}

.control-panel button {
  width: 100%;
  height: 25%;
}

.video-feeds {
  margin-left: 60px;
  width: calc(100% - 60px);
}

.camera-row {
  width: 100%;
  display: flex;
  justify-content: space-between;
}

.camera-container,
.camera-placeholder {
  flex: 1;
  margin: 0 10px;
  background-color: grey;
  text-align: center;
  line-height: calc((1/3 * 100vw - 60px) * 0.75);
  /* This helps vertically center the text */
}

.camera-container img {
  width: 100%;
  height: auto;
  max-height: calc((1/3 * 100vw - 60px) * 0.75);
  /* height based on aspect ratio */
  display: block;
  margin: auto;
}</style>
