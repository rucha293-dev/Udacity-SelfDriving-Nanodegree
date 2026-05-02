

# Sensor Fusion — LiDAR Point Cloud Analysis

## 🔹 Range Image Visualization (ID_S1_EX1)

![Range and Intensity](rangesol.png)

The range channel was converted to 8-bit to represent distance information, while the intensity channel highlights reflectivity differences. The image was cropped to the front 180° field of view and both channels were stacked vertically. Intensity helps identify reflective surfaces such as vehicles.

---

## 🔹 Point Cloud Visualization (ID_S1_EX2)

![Point Cloud](id_s1_ex2.sol.png)

The LiDAR point cloud was visualized using Open3D. The road appears as structured scan lines, while vehicles appear as clusters of points above the ground.

---

## 🔹 Vehicle Examples



Six vehicle examples were identified with varying visibility including close, far, partially occluded, and side-view vehicles.

---

## 🔹 Stable Vehicle Features

* Vehicles appear as **compact clusters above the ground**
* The **roof is flat and horizontal**
* The **front and rear form vertical edges**
* Vehicles often appear **elongated along their length**
* Point density decreases with distance

---

## 🔹 Intensity-Based Observations

* High intensity → reflective surfaces (e.g., rear bumper)
* Vehicles appear brighter than road
* Helps distinguish objects from background

---

## 🔹 Conclusion

LiDAR provides strong geometric and intensity cues for detecting vehicles. Their shape, elevation, and elongated structure make them identifiable across different distances.
