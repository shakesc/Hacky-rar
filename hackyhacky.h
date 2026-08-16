/*
  Hacky Controller
https://raw.githubusercontent.com/RuiSantosdotme/ESP32-Weather-Shield-PCB/master/Code/index.html
https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/
*/

const char html_page[] PROGMEM = R"RawString(
<!DOCTYPE html>
<html>
  <style>

    h1 {text-align: center; font-size: 40px;}
    p {text-align: center; color: #4CAF50; font-size: 40px;}

      body {
        text-align: center;
        font-family: "Trebuchet MS", Arial;
      }
      table {
        border-collapse: collapse;
        width:60%;
        margin-left:auto;
        margin-right:auto;
      }
      th {
        padding: 16px;
        background-color: #0043af;
        color: white;
      }
      tr {
        border: 1px solid #ddd;
        padding: 16px;
      }
      tr:hover {
        background-color: #bcbcbc;
      }
      td {
        border: none;
        padding: 16px;
      }
      .sensor {
        color:white;
        font-weight: bold;
        background-color: #bcbcbc;
        padding: 8px;
      }
    </style>

<body>
  <h1>Hacky Racers Control Board</h1>
<br>

<table>
      <tr>
        <th>SENSOR</th>
        <th>VALUE</th>
        <th>UNIT</th>

      </tr>
      <tr>
        <td><span class="sensor">Amp Inst</span></td>
        <td><span class="sensor" id="current_inst">...</span></td>
        <td> Amps</td>
      </tr>
      <tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="current_peak">...</span></td>
        <td> Amps</td>
      </tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="temp_inst">...</span></td>
        <td> *C</td>
      </tr>
      <tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="temp_peak">...</span></td>
        <td> *C</td>
      </tr>
      <tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="speed_current">...</span></td>
        <td> KmPH</td>
      </tr>
      <tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="speed_peak">...</span></td>
        <td> KmPH</td>
      </tr>
      <tr>
        <td><span class="sensor">Amp Peak</span></td>
        <td><span class="sensor" id="DistanceTravelledMeter">...</span></td>
        <td> Metres</td>
      </tr>
</table>


<script>
  setInterval(function() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        const text = this.responseText;
        const myArr = JSON.parse(text);
        document.getElementById("current_inst").innerHTML = myArr[0];
        document.getElementById("current_peak").innerHTML = myArr[1];
        document.getElementById("temp_inst").innerHTML = myArr[2];
        document.getElementById("temp_peak").innerHTML = myArr[3];
        document.getElementById("speed_current").innerHTML = myArr[4];
        document.getElementById("speed_peak").innerHTML = myArr[5];
        document.getElementById("DistanceTravelledMeter").innerHTML = myArr[6];
      }
    };
    xhttp.open("GET", "readHackyControl", true);
    xhttp.send();
  },100);
</script>
</body>
</html>
)RawString";
