```javascript
function Decoder(b, port) {

var temp = (b[1] << 8) | b[0];
var hr = b[2] | b[3] << 8;
var spo= ( b[4] | b[5] << 8 );
//var spo= (b[5] << 8) | b[4]/100;
var fall = (b[7] << 8) | b[6];

return {
TEMP: temp,
HeartBeat: hr,
SPO: spo,
FALL: fall
}
}
```
