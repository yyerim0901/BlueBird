<template>
  <v-app id="app">
    <NavBar />
    <router-view></router-view>
  </v-app>
</template>

<script>
import NavBar from './components/NavBar';

export default {
  name: 'App',

  components: {
    NavBar,
  },

  data: () => ({
    //
  }),
  mounted() {
    const socket = this.$socket
    socket.on("connect", () => {
      console.log("연결되었습니다."); // x8WIv7-mJelg7on_ALbx
    });
    window.onbeforeunload = function (event) {
        socket.emit('sendScanOffToServer', 'data')
        console.log(event)
      }
  }
};
</script>
<style>
#app {
  background-color: #FFFBE6 !important;
}

</style>