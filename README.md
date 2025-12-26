# CradleSense  
**IoT-Based Neonatal Care Monitoring System**

---

## 1. What is CradleSense?

**CradleSense** is an IoT-based healthcare monitoring system designed for **Neonatal Intensive Care Units (NICU)**.  
It continuously monitors vital parameters of newborn babies using sensors embedded in a smart cradle and displays the data in real time through dedicated dashboards for clinicians and parents.

The system aims to reduce dependency on manual monitoring, enable early detection of critical conditions, and improve transparency and reassurance for parents.

---

## 2. What CradleSense Does

CradleSense performs the following functions:

- Continuously monitors neonatal vital parameters  
- Transmits real-time sensor data to the cloud  
- Stores historical health data securely  
- Detects abnormal conditions using predefined threshold rules  
- Alerts medical staff instantly during critical situations  
- Displays health data through:
  - **Clinical Dashboard** (for doctors and nurses)
  - **Parental Dashboard** (for parents, read-only access)

---

## 3. Existing Solutions and Their Limitations

### Existing Solutions in NICU

Current NICUs primarily rely on:
- Bedside medical monitoring devices  
- Manual observation by nurses  
- Standalone monitors for vitals such as heart rate and oxygen saturation  

### Limitations of Existing Solutions

- High dependency on manual supervision  
- Limited or no real-time remote access  
- Lack of a unified monitoring dashboard  
- Parents cannot monitor the baby’s health remotely  
- Delayed response during staff overload  
- Expensive systems with limited scalability for smaller hospitals  

---

## 4. How CradleSense Overcomes These Limitations

| Problem in Existing System | CradleSense Solution |
|---------------------------|---------------------|
| Manual monitoring | Automated continuous sensing |
| Delayed alerts | Real-time cloud-based alerts |
| Isolated monitors | Unified dashboard view |
| No parental access | Dedicated parental dashboard |
| High cost | Low-cost IoT-based design |
| Poor scalability | Cloud-based scalable architecture |

---

## 5. Challenges Faced

During the design and development of CradleSense, the following challenges were identified:

- Accurate sensing of neonatal vitals using low-cost sensors  
- Ensuring reliable real-time data transmission  
- Designing a system that is safe and non-intrusive for newborns  
- Secure separation of clinical and parental access  
- Handling intermittent network connectivity  
- Designing dashboards that are informative yet easy to understand  
- Making the system modular and scalable for future expansion  

---

## 6. System Architecture

### 6.1 High-Level Architecture Diagram
<img width="2501" height="1518" alt="image" src="https://github.com/user-attachments/assets/606e4289-055c-42f5-a7e4-e9a3dab7f411" />

### 6.2 Hardware Block Diagram (Smart Cradle Side)
<img width="2572" height="1645" alt="image" src="https://github.com/user-attachments/assets/f3f51137-f280-495d-8dfc-50af177f0819" />

### 6.3 Software Architecture (Backend + Frontend)
<img width="4074" height="1658" alt="image" src="https://github.com/user-attachments/assets/f3eb9d1a-ed5a-42c8-9f8b-1c4923acd451" />

### 6.5 Data Flow Diagram
<img width="2076" height="1248" alt="mermaid-diagram-2025-12-26-153714" src="https://github.com/user-attachments/assets/a9abd994-a704-4cde-87e1-53d523f22df6" />

---

## 8. Hardware Implementation

### Hardware Components Used

- ESP32 Microcontroller  
- Temperature Sensor  
- Heart Rate Sensor  
- SpO₂ Sensor  
- Motion / Pressure Sensor  
- Power Supply Module  

### Hardware Functionality

- Sensors are placed safely within the cradle  
- ESP32 reads sensor values at fixed time intervals  
- Sensor data is validated and formatted into JSON  
- Data is transmitted wirelessly to the cloud backend  

---

## 9. Software Implementation

### Backend

- Receives sensor data from ESP32 devices  
- Stores data in a cloud-based database  
- Applies threshold-based rules for health evaluation  
- Generates alerts for abnormal readings  

### Dashboards

#### Clinical Dashboard
- Real-time monitoring of all babies  
- Alert notifications for critical conditions  
- Historical data visualization  
- Priority highlighting of high-risk cases  

#### Parental Dashboard
- Simplified health status view  
- Real-time vital parameters  
- Read-only access  
- Secure authentication  

---

## 10. Sample Output

### Sensor Data Output (Example)

```json
{
  "babyId": "NICU_01",
  "temperature": 36.9,
  "heartRate": 130,
  "spo2": 97,
  "movement": "normal",
  "timestamp": "2025-01-10T10:30:00Z"
}
```
### Dashboard Output
- Live graphs for temperature, heart rate, and SpO₂
- Alert messages on the clinical dashboard
- Health status indicators on the parental dashboard

