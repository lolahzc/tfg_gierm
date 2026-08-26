#!/usr/bin/env python3
"""Genera el modelo Gazebo 'mission_labels': carteles y marcas de suelo.

Lee config/conf.yaml (bases de carga, operarios, herramientas) y
config/mission.yaml (waypoints de cada tarea) y produce:

  gazebo_worlds/models/mission_labels/model.sdf
  gazebo_worlds/models/mission_labels/materials/textures/*.png

Se rotula con TEXTURAS, no con marcadores de texto: el renderer Ogre2 de
Gazebo Sim 8 no implementa el marcador TEXT (falla con "Invalid Marker
type [7]"), asi que la unica forma fiable de poner texto en el mundo es
hornearlo en una imagen y aplicarla sobre un plano.

Uso:
    python3 scripts/gen_mission_labels.py
    colcon build --packages-select mission_planner
"""
import math
import os
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
MODEL_DIR = os.path.join(PKG, 'gazebo_worlds', 'models', 'mission_labels')
TEX_DIR = os.path.join(MODEL_DIR, 'materials', 'textures')

FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
]

# Un color por dron, el mismo que usa mission_viz.py para la bola que
# sobrevuela cada uno: asi la base y su dron se identifican de un vistazo.
DRONE_COLORS = {
    'drone0': (220, 40, 40),
    'drone1': (40, 170, 60),
    'drone2': (50, 90, 220),
}
# Un color por tipo de tarea.
TASK_COLORS = {
    'I': (0, 160, 200),    # Inspect
    'A': (240, 140, 20),   # InspectPVArray
    'M': (200, 60, 160),   # Monitor
    'D': (150, 60, 200),   # DeliverTool
}
TASK_NAMES = {'I': 'INSPECCION', 'A': 'PANELES', 'M': 'VIGILAR', 'D': 'ENTREGA'}


def _font_path():
    for f in FONT_CANDIDATES:
        if os.path.exists(f):
            return f
    raise SystemExit('No se encontro ninguna fuente TrueType utilizable')


def make_label(name, text, rgb, w=1024, h=256):
    """Cartel blanco con borde de color y el texto centrado."""
    img = Image.new('RGBA', (w, h), (255, 255, 255, 255))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w - 1, h - 1], outline=rgb + (255,), width=16)
    size = int(h * 0.55)
    font = ImageFont.truetype(_font_path(), size)
    while size > 10:
        font = ImageFont.truetype(_font_path(), size)
        bb = d.textbbox((0, 0), text, font=font)
        if bb[2] - bb[0] <= w * 0.90 and bb[3] - bb[1] <= h * 0.70:
            break
        size -= 4
    bb = d.textbbox((0, 0), text, font=font)
    d.text(((w - (bb[2] - bb[0])) / 2 - bb[0], (h - (bb[3] - bb[1])) / 2 - bb[1]),
           text, font=font, fill=rgb + (255,))
    # Pre-giro que cancela el que aplica Gazebo sobre la cara superior del box.
    if TEXTURE_ROTATION_DEG:
        img = img.rotate(TEXTURE_ROTATION_DEG, expand=True)
    img.save(os.path.join(TEX_DIR, name + '.png'))


# La cara superior de un <box> en Gazebo mapea la textura girada 90 grados, asi
# que el texto salia escrito a lo ancho del lado CORTO del cartel (ilegible sin
# hacer mucho zoom). Girar el link entero con un yaw no arregla nada: gira la
# caja Y la textura a la vez, con lo que el texto sigue cruzado respecto al
# rectangulo. La solucion es pre-girar la IMAGEN para cancelar el giro que
# aplica Gazebo; asi la palabra queda escrita a lo largo del lado LARGO.
# Si en tu vista sale boca abajo, cambia 90 por -90 y recompila.
TEXTURE_ROTATION_DEG = 90

# El cartel se deja sin yaw: la orientacion la da ya la textura.
LABEL_YAW = 0.0

LABEL_SCALE = 1.0


def label_link(name, tex, x, y, z, sx, sy, yaw=LABEL_YAW):
    """Plano tumbado en el suelo con la textura del cartel."""
    sx *= LABEL_SCALE
    sy *= LABEL_SCALE
    return f"""
    <link name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <visual name="v">
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx:.3f} {sy:.3f} 0.02</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>model://mission_labels/materials/textures/{tex}.png</albedo_map>
            <metalness>0.0</metalness><roughness>1.0</roughness>
          </metal></pbr>
        </material>
      </visual>
    </link>"""


def shape_link(name, kind, x, y, z, rgb, radius=1.0, length=1.0, alpha=1.0):
    """Marca de color solida (cilindro o esfera), sin colision."""
    r, g, b = [c / 255.0 for c in rgb]
    if kind == 'cylinder':
        geom = f'<cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder>'
    else:
        geom = f'<sphere><radius>{radius:.3f}</radius></sphere>'
    return f"""
    <link name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <visual name="v">
        <cast_shadows>false</cast_shadows>
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {alpha:.2f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {alpha:.2f}</diffuse>
          <emissive>{r*0.35:.3f} {g*0.35:.3f} {b*0.35:.3f} 1</emissive>
        </material>
      </visual>
    </link>"""


def main():
    os.makedirs(TEX_DIR, exist_ok=True)
    conf = yaml.safe_load(open(os.path.join(PKG, 'config', 'conf.yaml')))
    mission = yaml.safe_load(open(os.path.join(PKG, 'config', 'mission.yaml')))

    links = []

    # --- Bases de carga -------------------------------------------------
    stations = conf.get('positions', {}).get('charging_stations', {})
    for st_name, pos in stations.items():
        drone = st_name.replace('charging_station_', '')
        # charging_station_4 / _5 vienen de conf.yaml y no son de ningun dron
        # concreto: la 4 es el fallback de BackToStation para cualquier agente
        # que no sea drone0/1/2, y la 5 no la usa nadie. Se pintan en gris para
        # que no se confundan con las bases asignadas.
        assigned = drone in DRONE_COLORS
        color = DRONE_COLORS.get(drone, (130, 130, 130))
        x, y = float(pos['x']), float(pos['y'])
        # Plataforma circular del color del dron.
        links.append(shape_link(f'pad_{drone}', 'cylinder', x, y, 0.03, color,
                                radius=1.6, length=0.05))
        # Anillo interior mas claro, para que se vea el centro exacto.
        links.append(shape_link(f'pad_in_{drone}', 'cylinder', x, y, 0.05,
                                tuple(min(255, c + 70) for c in color),
                                radius=0.55, length=0.06))
        text = f'BASE {drone.upper()}' if assigned else f'CARGA LIBRE {drone}'
        make_label(f'base_{drone}', text, color)
        links.append(label_link(f'lbl_base_{drone}', f'base_{drone}',
                                x, y - 3.6, 0.03, 4.0, 1.0))

    # --- Operarios (human targets) --------------------------------------
    for i, (hname, pos) in enumerate(sorted(conf.get('human_targets', {}).items()), 1):
        x, y = float(pos['x']), float(pos['y'])
        color = (240, 140, 20)
        links.append(shape_link(f'ring_{hname}', 'cylinder', x, y, 0.03, color,
                                radius=2.2, length=0.04))
        make_label(f'lbl_{hname}', f'OPERARIO {i}', color)
        links.append(label_link(f'l_{hname}', f'lbl_{hname}', x, y - 4.2, 0.03, 4.4, 1.1))

    # --- Herramientas ---------------------------------------------------
    tools = conf.get('tools', {})
    if tools:
        xs = [float(t['x']) for t in tools.values()]
        ys = [float(t['y']) for t in tools.values()]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        color = (150, 60, 200)
        for tname, t in tools.items():
            links.append(shape_link(f'tool_{tname}', 'sphere',
                                    float(t['x']), float(t['y']), 0.35, color, radius=0.32))
        make_label('lbl_tools', 'HERRAMIENTAS', color)
        links.append(label_link('l_tools', 'lbl_tools', cx, cy - 3.0, 0.03, 5.0, 1.25))

    # --- Waypoints de cada tarea de la mision ---------------------------
    n_wp = 0
    for wave in mission.get('waves', []):
        for task in wave.get('tasks', []):
            ttype = task['type'].upper()
            tid = task['id']
            color = TASK_COLORS.get(ttype, (120, 120, 120))
            wps = task.get('waypoints', [])
            for j, wp in enumerate(wps):
                x, y, z = float(wp['x']), float(wp['y']), float(wp['z'])
                # Poste vertical desde el suelo hasta la altura pedida: se ve
                # de lejos y ademas ensena a que altura va el dron.
                links.append(shape_link(f'post_{tid}_{j}', 'cylinder', x, y, z / 2.0,
                                        color, radius=0.16, length=max(z, 0.2)))
                links.append(shape_link(f'top_{tid}_{j}', 'sphere', x, y, z, color, radius=0.45))
                n_wp += 1
            if wps:
                x0, y0 = float(wps[0]['x']), float(wps[0]['y'])
                make_label(f'lbl_{tid}', f'{tid}  {TASK_NAMES.get(ttype, ttype)}', color)
                links.append(label_link(f'l_{tid}', f'lbl_{tid}', x0, y0 - 3.2, 0.03, 4.6, 1.15))

    sdf = f"""<?xml version="1.0" ?>
<!-- GENERADO por scripts/gen_mission_labels.py - no editar a mano.
     Regenerar tras cambiar config/mission.yaml o config/conf.yaml. -->
<sdf version="1.8">
  <model name="mission_labels">
    <static>true</static>{''.join(links)}
  </model>
</sdf>
"""
    open(os.path.join(MODEL_DIR, 'model.sdf'), 'w').write(sdf)
    open(os.path.join(MODEL_DIR, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model>\n  <name>mission_labels</name>\n'
        '  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n'
        '  <description>Carteles y marcas de suelo de la mision '
        '(generado por gen_mission_labels.py).</description>\n</model>\n')

    print(f'mission_labels: {len(links)} links, {n_wp} waypoints, '
          f'{len(stations)} bases, {len(tools)} herramientas')


if __name__ == '__main__':
    sys.exit(main())
