<template>
    <div>
        <Header />
        <div class="mp_full container">
            <div class="mp_full container">
                <h4 class="del_text">디바이스</h4>
            </div>
            <div class="card rounded-3 card_size">
                <div class="card-body">
                    <h5 class="card-title">장소</h5>
                    <select 
                    class="form-select" 
                    aria-label="Default select example"
                    @change="onChange($event)">
                        <option selected class="placeholder">장소를 선택해주세요.</option>
                        <option value="1">세미나실</option>
                        <option value="2">창고</option>
                        <option value="3">회의실</option>
                        <option value="4">사무실</option>
                        <option value="5">사장실</option>
                    </select>
                </div>
                <div class="container">
                    <div class="row">
                        <div class="ct_text col" style="text-align:center;">
                            <p class="print_text">{{this.value.device_name}}</p>
                        </div>
                        <div class="switch_pad col" v-show="showtext">
                        <!-- <div class="d-flex justify-content-center col form-check form-switch">
                            <input class="form-check-input" type="checkbox" id="flexSwitchCheckChecked" checked>
                        </div>     -->
                        <img class="img_set" @click="on" src="../assets/img/on.png" alt="">
                        <img class="img_set" @click="off" src="../assets/img/off.png" alt="">
                        </div>
                    </div>
                </div>
            </div>
        </div>
        <div class="footer">
            <Footer />
        </div>
    </div>
</template>

<script>
import Header from "./Header.vue"
import Footer from "./Footer.vue"

export default {
    name: 'Control',
    components: { Header, Footer },
    data() {
        return {
            value:{
                room_name:"",
                device_name :"",
                on_off: ""
            },
            selectvalue:"",
            showtext:false,
            available: null,
        }
    },
    created() {
        this.$socket.on('bot_status_response_web', (data) => {
            this.available = data['available']
        })
    },
    methods: {
        onChange(event){
            this.selectvalue = event.target.value;

            switch (this.selectvalue) {
                case "1":
                    this.value.room_name = "세미나";
                    this.value.device_name = "공기 청정기";
                    this.showtext = true;
                    break;
                case "2":
                    this.value.room_name = "창고";
                    this.value.device_name = "해당 장소에 제어할 수 있는 기기가 없습니다.";
                    this.showtext = false;
                    break;
                case "3":
                    this.value.room_name = "회의";
                    this.value.device_name = "에어컨";
                    this.showtext = true;
                    break;
                case "4":
                    this.value.room_name = "사무";
                    this.value.device_name = "에어컨";
                    this.showtext = true;
                    break;
                case "5":
                    this.value.room_name = "사장";
                    this.value.device_name = "TV";
                    this.showtext = true;
                    break;
                default:
                    this.value.room_name = "default";
                    this.value.device_name = "해당 장소에 제어할 수 있는 기기가 없습니다.";
                    this.showtext = false;
                    break;
            }
        },
        on(){
            if (this.available != '사용가능') {
                alert('터틀봇이 다른 명령을 수행하고 있습니다')
                return
            }
            //상태를 받고 on 전달
            this.value['on_off'] = 'on'
            this.$socket.emit('deviceControl',this.value);
        },
        off(){
            if (this.available != '사용가능') {
                alert('터틀봇이 다른 명령을 수행하고 있습니다')
                return
            }
            this.value['on_off'] = 'off'
            this.$socket.emit('deviceControl',this.value);
        },
    },
    watch:{
        curDeviceState(){
            //화면 값이 바뀔 때 마다 숫자나 텍스트를 전달해서 status받아오기
        }
    }
}
</script>

<style>
.switch_pad{
    margin: 10px;
    padding:10px;
}
.ct_text{
    text-align: center;
}
.print_text{
    padding-top:28px;
    margin-bottom: 40px;
    /* padding: 10px;
    margin-left:30px;
    margin-top: 9px;
    margin-right: 9px;
    margin-bottom: 9px; */
}
.img_set{
    width : 50px;
    padding-top: 0px;
    padding-left: 0px;
}
</style>