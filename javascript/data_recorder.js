
imu_data = {}
imu_data["accelerometer"] = {}
imu_data["gyroscope"] = {}
let acl = new Accelerometer({frequency: 60});

acl.addEventListener('reading', () => {
    document.getElementById("ax").innerHTML = "ax: "+acl.x.toFixed(7);
    document.getElementById("ay").innerHTML = "ay: "+acl.y.toFixed(7);
    document.getElementById("az").innerHTML = "az: "+acl.z.toFixed(7);
    document.getElementById("ta").innerHTML = "ta: "+acl.timestamp;
    imu_data["accelerometer"][acl.timestamp] = [acl.x,acl.y,acl.z]
});
acl.start();
let gyr = new Gyroscope({frequency: 60});
gyr.addEventListener('reading', () => {
    document.getElementById("gx").innerHTML = "gx: "+gyr.x.toFixed(7);
    document.getElementById("gy").innerHTML = "gy: "+gyr.y.toFixed(7);
    document.getElementById("gz").innerHTML = "gz: "+gyr.z.toFixed(7);
    document.getElementById("tg").innerHTML = "tg: "+gyr.timestamp;
    imu_data["gyroscope"][gyr.timestamp] = [gyr.x,gyr.y,gyr.z]
});
gyr.start();


// Function to download data to a file
function download(data=imu_data, filename="test.json", type="text/plain") {
    var dbParam = JSON.stringify(data);
    var file = new Blob([dbParam], {type: type});
    if (window.navigator.msSaveOrOpenBlob) // IE10+
        window.navigator.msSaveOrOpenBlob(file, filename);
    else { // Others
        var a = document.createElement("a"),
                url = URL.createObjectURL(file);
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        setTimeout(function() {
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);  
        }, 0); 
    }
}

// var x = document.getElementById("demo");
// function getLocation() {
//   if (navigator.geolocation) {
//     navigator.geolocation.getCurrentPosition(showPosition);
//   } else {
//     x.innerHTML = "Geolocation is not supported by this browser.";
//   }
// }

// function showPosition(position) {
//   x.innerHTML = "Latitude: " + position.coords.latitude +
//   "<br>Longitude: " + position.coords.longitude+
//   "<br>Accuracy: " + position.coords.accuracy +
//   "<br>Altitude: " + position.coords.altitude +
//   "<br>Altitude accuracy: " + position.coords.altitudeAccuracy +
//   "<br>Timestamp: " + position.coords.timestamp;
// }