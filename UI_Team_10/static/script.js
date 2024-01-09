
/*
// Heart Rate
fetch('https://api.thingspeak.com/channels/2165858/fields/1.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field1));
        values.forEach(alert_death => {
            if (alert_death < 70) {
                alertMessages.push("Heart Rate is falling down, check the patient");
                
            }
        });
    })

// SPO2 Chart
fetch('https://api.thingspeak.com/channels/2165858/fields/2.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field2));
        values.forEach(alert_death => {
            if (alert_death < 94) {
                alertMessages.push("SPO2 Readings are abnormal, check the patient");
                break;
            }
        });
    })

// Systolic Pressure
fetch('https://api.thingspeak.com/channels/2165858/fields/3.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field3));
        values.forEach(alert_death => {
            if (alert_death > 140) {
                alertMessages.push("Systolic Pressure is abnormal, check the patient");
            }
        });
    })

// Diastolic Pressure
fetch('https://api.thingspeak.com/channels/2165858/fields/4.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field4));
        values.forEach(alert_death => {
            if (alert_death > 100) {
                alertMessages.push("Diastolic Pressure is abnormal, check the patient");
            }
        });
    })

// Body Temperature
fetch('https://api.thingspeak.com/channels/2165858/fields/5.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field5));
        values.forEach(alert_death => {
            if (alert_death > 99) {
                alertMessages.push("Body Temperature of the patient is above 98.6 F, check them.");
            }
        });
    })

// Saline Level
fetch('https://api.thingspeak.com/channels/2165858/fields/6.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field6));
        values.forEach(alert_death => {
            if (alert_death > 25) { //change this when saline level sensor readings are actually calculated
                alertMessages.push("Saline bottle needs to be refilled");
            }
        });
        displayAlerts(alertMessages);
    });

function displayAlerts(alertMessages) {
    if (alertMessages.length > 0) {
        alert(alertMessages.join("\n"));
        alertMessages.length = 0; // Clear the alertMessages array
    }
}

function createChart(canvasId, label, labels, values) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    var cht = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: label,
                data: values,
                fill: false,
                borderColor: 'rgb(75, 192, 192)',
                tension: 0.1
            }]
        },
        options: {
            scales: {
                x: {
                    display: false
                }
            }
        }
    });
}
*/

var alertMessages = [];

// Heart Rate
fetch('https://api.thingspeak.com/channels/2165858/fields/1.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field1));
        for (const alert_death of values) {
            if (alert_death < 65) {
                alertMessages.push("Heart Rate is falling down, check the patient");
                break; // Exit the loop after pushing the alert message
            }
            else if(alert_death > 100){
                alertMessages.push("Heart Rate is abnormally above the normal range, check the patient");
                break;
            }
        }
    })

// SPO2 Chart
fetch('https://api.thingspeak.com/channels/2165858/fields/2.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field2));
        for (const alert_death of values) {
            if (alert_death < 94) {
                alertMessages.push("SPO2 Readings are abnormally low, check the patient");
                break; // Skip to the next alert condition
            }
            
        }
    })

// Systolic Pressure
fetch('https://api.thingspeak.com/channels/2165858/fields/3.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field3));
        for (const alert_death of values) {
            if (alert_death > 140) {
                alertMessages.push("Systolic Pressure is abnormally high, check the patient for hypertension");
                break; // Skip to the next alert condition
            }
            else if(alert_death <139 && alert_death>120){
                alertMessages.push("Systolic Pressure is at risk, check the patient");
                break;
            }

        }
    })

// Diastolic Pressure
fetch('https://api.thingspeak.com/channels/2165858/fields/4.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field4));
        for (const alert_death of values) {
            if (alert_death > 90) {
                alertMessages.push("Diastolic Pressure is abnormal high, check the patient for hypertension");
                break; // Skip to the next alert condition
            }
            else if(alert_death <89 && alert_death>80){
                alertMessages.push("Diastolic Pressure is at risk, check the patient");
                break;
            }
        }
    })

// Body Temperature
fetch('https://api.thingspeak.com/channels/2165858/fields/5.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field5));
        for (const alert_death of values) {
            if (alert_death > 99) {
                alertMessages.push("Body Temperature of the patient is above 99°F, cater for the high temperature");
                break; // Skip to the next alert condition
            }
            else if(alert_death<97){
                alertMessages.push("Body Temperature of the patient is below 97°F, cater for the low temperature");
                break;
            }
        }
    })

// Saline Level
fetch('https://api.thingspeak.com/channels/2165858/fields/6.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field6));
        for (const alert_death of values) {
            if (alert_death > 25) { //change this when saline level sensor readings are actually calculated
                alertMessages.push("Saline bottle needs to be refilled");
                break;
            }
        }
        // displayAlerts(alertMessages);
    });

//Flex sensor reading thingspeak
    fetch('https://api.thingspeak.com/channels/2165858/fields/7.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field7));
        for (const alert_death of values) {
            if (alert_death>1) { //change this when flex sensor readings are actually calculated
                alertMessages.push("Flex Sensor has detected abrupt movement. The patient might not be fine. Have a look at them");
                break;
            }
        }
        // displayAlerts(alertMessages);
    });

//Accelerometer Reading Thingspeak
fetch('https://api.thingspeak.com/channels/2165858/fields/8.json')
    .then(response => response.json())
    .then(data => {
        const values = data.feeds.map(feed => parseFloat(feed.field8));
        for (const alert_death of values) {
            if (alert_death > 1) { //change this when accelerometer sensor readings are actually calculated
                alertMessages.push("Accelerometer has detected abrupt motion. Look at the patient as they might not be fine");
                break;
            }
        }
        displayAlerts(alertMessages);
    });

function displayAlerts(alertMessages) {
    if (alertMessages.length > 0) {
        alert(alertMessages.join("\n"));
        alertMessages.length = 0; // Clear the alertMessages array
    }
}

function createChart(canvasId, label, labels, values) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    var cht = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: label,
                data: values,
                fill: false,
                borderColor: 'rgb(75, 192, 192)',
                tension: 0.1
            }]
        },
        options: {
            scales: {
                x: {
                    display: false
                }
            }
        }
    });
}
