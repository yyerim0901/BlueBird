const express = require('express');
const Device = require('../models/Device');

module.exports = {
    doGetDevices : function(req, res, next){
        const room_number = req.params.roomNumber;
        const data = {
            'room_number' : room_number
        }

        console.log('search device list in #room : ', room_number);
        Device.getDevices(data).then((result)=>{
            res.send(result);
        })
    }
    ,
    OnDevice : function(req, res, next){
        const data = {
            'device_number' : req.body.device_number
        }
        // ROS device on 메시지 전송
        req.io.emit('msg','device on!');

        console.log('turn on #device : ', req.body.device_number);
        Device.onDevice(data).then((result)=>{
            res.send(result);
        });
    },
    OffDevice : function(req, res, next){
        const data = {
            'device_number' : req.body.device_number
        }
        // ROS device off 메세지 전송
        req.io.emit('msg', 'device off!');
        
        console.log('turn off #device : ', req.body.device_number);
        Device.offDevice(data).then((result)=>{
            res.send(result);
        });
    }
}