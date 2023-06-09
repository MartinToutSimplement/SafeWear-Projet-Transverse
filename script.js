let valueSlide = 1000;
document.querySelector('.tableau').style.display = 'none';
var chute_bool = false;
var ctx = document.getElementById('myChart').getContext('2d');
var myChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Température',
            data: [],
            borderColor: 'rgba(0, 180, 240, 1)',
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true
            }
        }
    }
});

var ctxh = document.getElementById('myChartHumidity').getContext('2d');
var myChartHumidity = new Chart(ctxh, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Humidité',
            data: [],
            borderColor: 'rgba(0, 180, 240, 1)',
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true
            }
        }
    }
});

let lastAccelerationUpdate = 0;
let lastAccelerationUpdateH = 0;
let humidityCharacteristic;
let temperatureCharacteristic;
let accelerationCharacteristic;

document.querySelector('#connectButton').addEventListener('click', function () {
    navigator.bluetooth.requestDevice({
            filters: [{
                name: 'SafeWear'
                // services: [0x181C]
            }],
            optionalServices: [0x181C]
        })
        .then(device => device.gatt.connect())
        .then(server => server.getPrimaryService(0x181C))
        .then(service => {
            service.getCharacteristic(0x2A6F)
                .then(characteristic => {
                    temperatureCharacteristic = characteristic;
                    temperatureCharacteristic.startNotifications();
                    temperatureCharacteristic.addEventListener('characteristicvaluechanged',
                        handleTemperatureChanged);
                });
            service.getCharacteristic(0x2A6E)
                .then(characteristic => {
                    accelerationCharacteristic = characteristic;
                    accelerationCharacteristic.startNotifications();
                    accelerationCharacteristic.addEventListener('characteristicvaluechanged',
                        handleAccelerationChanged);
                });
            service.getCharacteristic(0x2A6D)
                .then(characteristic => {
                    humidityCharacteristic = characteristic;
                    humidityCharacteristic.startNotifications();
                    humidityCharacteristic.addEventListener('characteristicvaluechanged',
                        handleHumidityChanged);
                })
                .then(connected => {
                    bluetoothConnected();
                });
        });
});

function handleTemperatureChanged(event) {

    let currentTime = new Date().getTime();
    if (currentTime - lastAccelerationUpdate > valueSlide) {
        lastAccelerationUpdate = currentTime;
        let temperature = event.target.value.getFloat32(0, true);
        let res = Math.round(temperature * 10) / 10
        // document.querySelector('#outputTemp').innerHTML = 'Température: ' + res + '°C';
        addData(myChart, '', res);
        if (res > 30 || res < 10) {
            document.querySelector('.alertTemp').style.display = 'block';
        } else {

        }
    }
}

function handleAccelerationChanged(event) {
    let acceleration = event.target.value.getFloat32(0, true);
    let res = Math.round(acceleration * 100) / 100
    // document.querySelector('#outputAccel').innerHTML = '<br>Accélération: ' + res + 'm/s²';
    if (res > 140) {
        chute_bool = true;
        document.querySelector('.alertChute').style.display = 'block';
        // document.querySelector('#outputAccel').style.color = 'red';
    } else {
        // document.querySelector('.alert').style.display = 'none';
        // document.querySelector('#outputAccel').style.color = 'black';
    }
}

function handleHumidityChanged(event) {
    let currentTime = new Date().getTime();
    if (currentTime - lastAccelerationUpdateH > valueSlide) {
        lastAccelerationUpdateH = currentTime;
        let humidity = event.target.value.getFloat32(0, true);
        let res_humidity = Math.round(humidity * 100) / 100;
        // console.log(res_humidity);
        addData(myChartHumidity, '', res_humidity);
        if (res_humidity > 80 || res_humidity < 20) {
            document.querySelector('.alertHumidity').style.display = 'block';
        } else {}
    }
}



function display_alert() {
    document.querySelector('.alertChute').style.display = 'block';
}
document.querySelector('#okButtonChute').addEventListener('click', function () {
    document.querySelector('.alertChute').style.display = 'none';
    document.querySelector('.alertTemp').style.display = 'none';
    document.querySelector('.alertHumidity').style.display = 'none';
});

document.querySelector('#okButtonTemp').addEventListener('click', function () {
    document.querySelector('.alertTemp').style.display = 'none';
});
document.querySelector('#okButtonHumidity').addEventListener('click', function () {
    document.querySelector('.alertHumidity').style.display = 'none';
});

function bluetoothConnected() {
    console.log("Connecté!!!");
    document.querySelector('#connectButton').disabled = true;
    document.querySelector('#connectButton').innerHTML = 'Connecté à SafeWear';
    document.querySelector('.tableau').style.display = 'block';
}

// function addData(chart, label, data) {
//   chart.data.labels.push(label);
//   chart.data.datasets.forEach((dataset) => {
//     dataset.data.push(data);
//   });
//   chart.update();
// }

function addData(chart, label, data) {
    // Add the new label and data to the chart's dataset
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(data);
    });

    // Check the number of labels in the dataset
    if (chart.data.labels.length > 20) {
        // Remove the first label and data from the dataset
        chart.data.labels.shift();
        chart.data.datasets.forEach((dataset) => {
            dataset.data.shift();
        });
    }

    // Update the chart
    chart.update();
}

function rangeSlide(value) {
    document.getElementById('rangeValue').innerHTML = value;
    valueSlide = value * 1000;
}
//afficher valeur de chute_bool
// console.log(chute_bool);

// if (chute_bool == true) {
//   display_alert();
// }else{
//   document.querySelector('.alert').style.display = 'none';
// }