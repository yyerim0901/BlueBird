const express = require('express');
const cookieParser = require('cookie-parser');
const morgan = require('morgan');
const app = express();
app.set('port', 3000);

const employeeRouter = require('./routes/employee');
const deviceRouter = require('./routes/device');

//요청 정보 확인
app.use(morgan('dev'));
//요청 cookie 파싱
app.use(cookieParser());

app.use('/device', deviceRouter);
app.use('/employee', employeeRouter);

app.listen(app.get('port'), ()=>{
    console.log('listen...port : ', app.get('port'));
});