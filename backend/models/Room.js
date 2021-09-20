const mysql = require('mysql');
const db = require('../config/db');
const table = 'room';

module.exports = {

    getEmployee : function(room_number){
        return new Promise((resolve, reject)=>{
            const con = mysql.createConnection(db);

            con.query(
                `select room_number, name, x, y
                from ${table}
                where employee_number = ${room_number}`, (err, result, fields)=>{
                    if(err){
                        reject(err);
                    }else{
                        resolve(result);
                    }
                }
            );
            con.end();
        });
    }
}