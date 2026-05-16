#!/usr/bin/env python3
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
GAZEBO_MODELS = ROOT / "src" / "rov_gamantaray_gazebo" / "models"


MODEL_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="kki_payload_{code}">
    <static>true</static>
    <link name="payload_link">
      <inertial>
        <mass>0.30</mass>
        <inertia>
          <ixx>0.00028</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00028</iyy>
          <iyz>0</iyz>
          <izz>0.00008</izz>
        </inertia>
      </inertial>
      <collision name="body_collision">
        <geometry><box><size>0.06 0.06 0.10</size></box></geometry>
      </collision>
      <visual name="body_visual">
        <geometry><box><size>0.06 0.06 0.10</size></box></geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
        </material>
      </visual>
      <visual name="qr_top">
        <pose>0 0 0.052 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.08 0.08</size>
          </plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>model://kki_payload_{code}/materials/textures/qr_{code}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>kki_payload_{code}</name>
  <version>0.1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Ammar</name><email>ammar@example.com</email></author>
  <description>KKI ROV payload with QR code {code}.</description>
</model>
"""


COLORS = {
    "A": ("0.9 0.12 0.10 1", "0.9 0.12 0.10 1"),
    "B": ("0.10 0.60 0.25 1", "0.10 0.60 0.25 1"),
    "C": ("0.15 0.25 0.95 1", "0.15 0.25 0.95 1"),
    "D": ("0.95 0.70 0.10 1", "0.95 0.70 0.10 1"),
}


def main() -> None:
    encoder = cv2.QRCodeEncoder_create()
    for code, (ambient, diffuse) in COLORS.items():
        model_dir = GAZEBO_MODELS / f"kki_payload_{code}"
        texture_dir = model_dir / "materials" / "textures"
        texture_dir.mkdir(parents=True, exist_ok=True)

        qr = encoder.encode(code)
        qr = cv2.resize(qr, (512, 512), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(str(texture_dir / f"qr_{code}.png"), qr)

        (model_dir / "model.sdf").write_text(
            MODEL_TEMPLATE.format(code=code, ambient=ambient, diffuse=diffuse),
            encoding="utf-8",
        )
        (model_dir / "model.config").write_text(
            MODEL_CONFIG_TEMPLATE.format(code=code),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
