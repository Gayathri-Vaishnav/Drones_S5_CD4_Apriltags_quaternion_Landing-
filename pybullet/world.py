"""PyBullet urban scene, ten addressed pads, moving vehicles and tag textures."""
from __future__ import annotations
import math
from pathlib import Path
import cv2
import numpy as np
import pybullet as p
import pybullet_data
from config import ADDRESSES, DT, LARGE_TAG_M, SMALL_TAG_M, PAD_SIZE_M

ROOT = Path(__file__).resolve().parent
TAG_DIR = ROOT / "assets" / "generated_tags"

def tag_texture(tag_id: int) -> str:
    TAG_DIR.mkdir(parents=True, exist_ok=True)
    path = TAG_DIR / f"tag36h11_{tag_id}.png"
    image = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE) if path.exists() else None
    if image is None or image.shape != (440,440):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, int(tag_id), 360)
        canvas = np.full((440, 440), 255, np.uint8)
        canvas[40:400, 40:400] = marker
        cv2.imwrite(str(path), canvas)
    return str(path)

def tag_surface_mesh(size: float) -> int:
    """Create a flat UV-mapped tag surface for a generated tag36h11 texture."""
    half=size/2
    vertices=((-half,-half,.002),(half,-half,.002),(half,half,.002),(-half,half,.002))
    indices=(0,1,2,0,2,3)
    # Crop the 40 px white margin in the 440 px source image so its black
    # marker border spans exactly `size`, matching the solvePnP tag geometry.
    lo,hi=40/440,400/440
    uvs=((lo,lo),(hi,lo),(hi,hi),(lo,hi))
    visual=p.createVisualShape(p.GEOM_MESH,vertices=vertices,indices=indices,uvs=uvs,
                               rgbaColor=(1,1,1,1))
    return visual

def box(half_extents, xyz, color, mass=0, rgba=None, collision=True):
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba or color)
    collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents) if collision else -1
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual, basePosition=xyz)

class UrbanWorld:
    def __init__(self, client: int):
        self.client = client
        self.pad_ids: dict[int, int] = {}
        self.vehicle_ids: dict[int, int] = {}
        self.vehicle_origins: dict[int, tuple[float, float, float]] = {}
        self.vehicle_z_offsets: dict[int,float] = {}
        self.vehicle_details: dict[int, list[tuple[int,np.ndarray,np.ndarray]]] = {}
        self.tag_plates: dict[int, tuple[int,int]] = {}
        self.tag_texture_ids: dict[int,int] = {}
        self._build()

    def _build(self):
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setTimeStep(DT, physicsClientId=self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        # Plain matte ground avoids the default PyBullet checkerboard plane texture.
        box((50,50,.1),(0,0,-.1),(.27,.29,.30,1))
        sky=p.createVisualShape(p.GEOM_BOX,halfExtents=(150,.08,60),rgbaColor=(.62,.73,.84,1),physicsClientId=self.client)
        p.createMultiBody(0,-1,sky,(0,48,25),physicsClientId=self.client)
        # Long asphalt street, raised curbs, pale sidewalks, and a pedestrian crossing.
        road_x = 2.5
        box((4.1, 21, .035), (road_x, 7, -.005), (.105,.115,.13,1))
        for side in (-1, 1):
            x = road_x + side*5.25
            box((1.15, 21, .06), (x, 7, .02), (.76,.73,.65,1))
            box((.12, 21, .09), (road_x + side*4.18, 7, .06), (.88,.84,.73,1))
        for y in range(-13, 29, 3):
            box((.055, .72, .012), (road_x, y, .041), (.9,.89,.82,1),collision=False)
        for crossing_y in (-6, 7, 20):
            for x in np.arange(road_x-3.35, road_x+3.4, .72):
                box((.25, .075, .014), (float(x), crossing_y, .043), (.94,.93,.87,1),collision=False)
        # Dense avenue skyline. Glass panels and floor bands break up each tower face.
        self._build_tower_row(-13.4, (0.50,.54,.57,1), toward_road=1)
        self._build_tower_row(22.0, (0.48,.55,.60,1), toward_road=-1)
        for y in (-9, -3, 3, 9, 15, 21):
            for x in (-2.9, 7.9):
                box((.045,.045,1.65),(x,y,1.72),(.16,.17,.18,1),collision=False)
                box((.34,.09,.06),(x+(.27 if x<0 else -.27),y,3.34),(.95,.78,.45,1),collision=False)
        self._build_palms()
        # Roof platforms positioned at address heights.
        for n in (6, 8, 9):
            a = ADDRESSES[n]
            roof_z=a.xyz[2]-.36
            box((2.3,2.3,roof_z/2),(a.xyz[0],a.xyz[1],roof_z/2),(.47,.50,.53,1))
            for floor_z in np.arange(1.0,roof_z-.25,1.25):
                for side in (-1,1):
                    for offset in np.arange(-1.65,1.7,.72):
                        glass=(.19,.38,.48,1)
                        box((.018,.22,.34),(a.xyz[0]+side*2.32,a.xyz[1]+offset,float(floor_z)),glass,collision=False)
                        box((.22,.018,.34),(a.xyz[0]+offset,a.xyz[1]+side*2.32,float(floor_z)),glass,collision=False)
            box((2.3, 2.3, .18), (a.xyz[0],a.xyz[1],a.xyz[2]-.18), (.45,.48,.53,1))
        for n, a in ADDRESSES.items():
            if a.kind == "vehicle":
                self.vehicle_origins[n] = a.xyz
                truck=(n==3)
                half=(1.08,1.55,.14) if truck else (1.0,.55,.28)
                color=(.16,.20,.22,1) if truck else (.16,.38,.72,1)
                self.vehicle_z_offsets[n]=-.14 if truck else -.28
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color, physicsClientId=self.client)
                self.vehicle_ids[n] = p.createMultiBody(0, col, vis,
                    (a.xyz[0],a.xyz[1],a.xyz[2]+self.vehicle_z_offsets[n]), physicsClientId=self.client)
                p.changeDynamics(self.vehicle_ids[n], -1, lateralFriction=1.2, spinningFriction=.01,
                                 rollingFriction=.001, restitution=0.0, contactStiffness=20000.0,
                                 contactDamping=40.0, physicsClientId=self.client)
                self.vehicle_details[n]=[]
                if truck:
                    self._build_semitruck(n)
                else:
                    cabin=p.createVisualShape(p.GEOM_BOX,halfExtents=(.50,.43,.17),rgbaColor=(.12,.19,.24,1),physicsClientId=self.client)
                    # Keep the cabin clear of the centered landing deck and tags.
                    self._vehicle_part(n,cabin,(1.25,-.02,.39))
                    for wx in (-.60,.60):
                        for wy in (-.49,.49):
                            wheel=p.createVisualShape(p.GEOM_CYLINDER,radius=.17,length=.10,
                                rgbaColor=(.055,.06,.065,1),physicsClientId=self.client)
                            self._vehicle_part(n,wheel,(wx,wy,-.07),p.getQuaternionFromEuler((math.pi/2,0,0)))
                    windshield=p.createVisualShape(p.GEOM_BOX,halfExtents=(.38,.018,.12),
                        rgbaColor=(.30,.48,.57,1),physicsClientId=self.client)
                    self._vehicle_part(n,windshield,(1.25,-.47,.40))
            z = a.xyz[2] + .035
            top = p.createCollisionShape(p.GEOM_BOX, halfExtents=(PAD_SIZE_M/2,PAD_SIZE_M/2,.035), physicsClientId=self.client)
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=(PAD_SIZE_M/2,PAD_SIZE_M/2,.035), rgbaColor=(.08,.72,.78,1), physicsClientId=self.client)
            self.pad_ids[n] = p.createMultiBody(0, top, visual, (a.xyz[0],a.xyz[1],z), physicsClientId=self.client)
            p.changeDynamics(self.pad_ids[n], -1, lateralFriction=1.2, spinningFriction=.01,
                             rollingFriction=.001, restitution=0.0, contactStiffness=20000.0,
                             contactDamping=40.0, physicsClientId=self.client)
            # Large approach marker (id = address) and small terminal marker (id + 10).
            large = self._tag_plate(n, LARGE_TAG_M, (a.xyz[0]-.36,a.xyz[1],a.xyz[2]), z + .037)
            small = self._tag_plate(n+10, SMALL_TAG_M, a.xyz, z + .041)
            self.tag_plates[n]=(large,small)

    def set_active(self, number: int | None):
        """Highlight the selected landing pad in the rendered world."""
        for n, body in self.pad_ids.items():
            color = (.30,.78,.40,1) if n == number else (.08,.55,.62,1)
            p.changeVisualShape(body, -1, rgbaColor=color, physicsClientId=self.client)

    def _build_tower_row(self, x, facade, toward_road):
        towers = ((-7,12.0,1.25), (-2.5,16.0,1.45), (2.0,21.0,1.35),
                  (6.5,13.0,1.55), (11.0,18.0,1.3), (15.5,11.0,1.5),
                  (20.0,17.0,1.4), (24.5,13.0,1.45))
        for index,(y,height,depth) in enumerate(towers):
            width=1.2 if index%2 else 1.45
            tone=facade if index%2==0 else (.61,.66,.68,1)
            box((width,depth,height/2),(x,y,height/2),tone)
            # Street-facing blue glass bays separated by concrete mullions.
            face_x=x+toward_road*(width+.025)
            for floor_z in np.arange(1.1,height-.35,1.35):
                box((.025,depth-.12,.045),(face_x,y,floor_z-.49),(.72,.76,.75,1),collision=False)
                for wy in np.arange(y-depth+.28,y+depth-.2,.48):
                    glass=(.20,.38,.46,1) if (int(wy*10)+index)%3 else (.28,.46,.52,1)
                    box((.018,.18,.40),(face_x+toward_road*.035,float(wy),float(floor_z)),glass,collision=False)
            # Strong corner fins and a clean roof parapet.
            for edge in (-1,1):
                box((.045,.055,height/2),(face_x+toward_road*.06,y+edge*(depth-.08),height/2),(.75,.77,.75,1),collision=False)
            box((width+.04,depth+.04,.12),(x,y,height+.12),(.72,.74,.72,1),collision=False)

    def _vehicle_part(self, number, shape, offset, orientation=(0,0,0,1)):
        body=p.createMultiBody(0,-1,shape,(0,0,0),orientation,physicsClientId=self.client)
        self.vehicle_details[number].append((body,np.asarray(offset,dtype=float),np.asarray(orientation,dtype=float)))

    def _vehicle_box(self, number, half_extents, offset, color):
        shape=p.createVisualShape(p.GEOM_BOX,halfExtents=half_extents,rgbaColor=color,physicsClientId=self.client)
        self._vehicle_part(number,shape,offset)

    def _build_semitruck(self, number):
        """Procedural tractor cab and long low-deck trailer, with a landing pad on deck."""
        white=(.82,.83,.80,1); dark=(.10,.13,.15,1); glass=(.18,.28,.33,1)
        metal=(.48,.50,.49,1); amber=(.95,.48,.08,1); wood=(.37,.25,.13,1)
        # Trailer deck, frame, headboard, side rails and rear loading ramps.
        self._vehicle_box(number,(1.02,1.60,.045),(0,.05,.09),wood)
        self._vehicle_box(number,(1.08,.10,.13),(0,-1.58,.16),metal)
        for side in (-1,1):
            self._vehicle_box(number,(.045,1.55,.09),(side*1.03,.03,.18),metal)
            self._vehicle_box(number,(.07,.47,.06),(side*.88,1.82,.04),metal)
            # Amber running lights and metal tie-downs along each rail.
            for y in np.arange(-1.25,1.5,.55):
                self._vehicle_box(number,(.028,.055,.035),(side*1.08,float(y),.20),amber)
        for y in np.arange(-1.40,1.48,.27):
            self._vehicle_box(number,(.96,.018,.012),(0,float(y),.145),(.48,.34,.20,1))
        # Tractor chassis, fifth-wheel neck and white cab; cab faces toward -Y.
        self._vehicle_box(number,(.20,1.40,.12),(0,-1.95,.02),dark)
        self._vehicle_box(number,(.16,.72,.09),(0,-1.22,.12),metal)
        self._vehicle_box(number,(.91,.66,.56),(0,-3.05,.51),white)
        self._vehicle_box(number,(.88,.42,.24),(0,-3.91,.24),white)
        self._vehicle_box(number,(.94,.10,.12),(0,-4.25,.01),dark)
        self._vehicle_box(number,(.76,.025,.28),(0,-3.72,.65),glass)
        self._vehicle_box(number,(.36,.022,.23),(0,-4.03,.24),dark)
        # Side windows, mirrors, step, fuel tank, grille and headlights.
        for side in (-1,1):
            self._vehicle_box(number,(.022,.43,.26),(side*.915,-3.03,.67),glass)
            self._vehicle_box(number,(.022,.15,.18),(side*.916,-3.55,.65),glass)
            self._vehicle_box(number,(.035,.07,.10),(side*1.00,-3.64,.54),dark)
            self._vehicle_box(number,(.12,.52,.09),(side*.92,-2.18,-.02),metal)
            self._vehicle_box(number,(.07,.38,.10),(side*.98,-3.55,.03),metal)
            self._vehicle_box(number,(.06,.10,.08),(side*.78,-4.31,.06),(.95,.88,.56,1))
        self._vehicle_box(number,(.40,.025,.16),(0,-4.17,.10),(.24,.27,.28,1))
        # Six axle pairs, including the trailer bogie. Wheels sit at street level.
        wheel_shape=p.createVisualShape(p.GEOM_CYLINDER,radius=.27,length=.15,
            rgbaColor=(.045,.05,.055,1),physicsClientId=self.client)
        rim_shape=p.createVisualShape(p.GEOM_CYLINDER,radius=.125,length=.012,
            rgbaColor=(.57,.59,.58,1),physicsClientId=self.client)
        axle_y=(-3.62,-2.25,-1.75,1.05,1.50,1.95)
        axle_q=p.getQuaternionFromEuler((0,math.pi/2,0))
        for y in axle_y:
            for side in (-1,1):
                x=side*(1.00 if y>-1.0 else .94)
                self._vehicle_part(number,wheel_shape,(x,y,.07),axle_q)
                self._vehicle_part(number,rim_shape,(x+side*.081,y,.07),axle_q)

    def _build_palms(self):
        """Add a row of stylized but articulated palms along the outer walkways."""
        for x in (-3.15,8.15):
            for y in (-11,-5,1,7,13,19,25):
                trunk=p.createVisualShape(p.GEOM_CYLINDER,radius=.09,length=2.8,
                    rgbaColor=(.43,.28,.16,1),physicsClientId=self.client)
                p.createMultiBody(0,-1,trunk,(x,y,1.42),physicsClientId=self.client)
                crown=(x,y,2.85)
                core=p.createVisualShape(p.GEOM_SPHERE,radius=.19,rgbaColor=(.22,.42,.19,1),
                    physicsClientId=self.client)
                p.createMultiBody(0,-1,core,crown,physicsClientId=self.client)
                for angle in np.linspace(0,2*math.pi,7,endpoint=False):
                    shape=p.createVisualShape(p.GEOM_BOX,halfExtents=(.62,.075,.035),
                        rgbaColor=(.17,.37,.15,1),physicsClientId=self.client)
                    rotation=p.getQuaternionFromEuler((0,.38,float(angle)))
                    point=(x+.38*math.cos(angle),y+.38*math.sin(angle),3.0)
                    p.createMultiBody(0,-1,shape,point,rotation,physicsClientId=self.client)

    def _tag_plate(self, tag_id, size, xyz, z):
        board=size*10/8
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=(board/2,board/2,.001), rgbaColor=(1,1,1,1), physicsClientId=self.client)
        plate = p.createMultiBody(0, -1, vis, (xyz[0], xyz[1], z), physicsClientId=self.client)
        if tag_id not in self.tag_texture_ids:
            self.tag_texture_ids[tag_id]=p.loadTexture(tag_texture(tag_id),physicsClientId=self.client)
        visual=tag_surface_mesh(size)
        black=p.createMultiBody(0,-1,visual,(xyz[0],xyz[1],z),physicsClientId=self.client)
        p.changeVisualShape(black,-1,textureUniqueId=self.tag_texture_ids[tag_id],physicsClientId=self.client)
        return plate,black

    def target_position(self, number: int, t: float) -> np.ndarray:
        a = ADDRESSES[number]
        x,y,z = a.xyz
        if a.motion == "line":
            if number == 3:
                y += 1.5 * math.sin(a.speed*t)
            else:
                x += 1.0 * math.sin(a.speed*t)
        elif a.motion == "circle":
            x += math.cos(a.speed*t)
            y += math.sin(a.speed*t)
        return np.array((x,y,z), dtype=float)

    def target_velocity(self, number: int, t: float) -> np.ndarray:
        a=ADDRESSES[number]
        if a.motion == "line":
            if number == 3: return np.array((0,1.5*a.speed*math.cos(a.speed*t),0))
            return np.array((a.speed*math.cos(a.speed*t),0,0))
        if a.motion == "circle": return np.array((-a.speed*math.sin(a.speed*t),a.speed*math.cos(a.speed*t),0))
        return np.zeros(3)

    def update(self, t: float):
        for n, body in self.vehicle_ids.items():
            xyz = self.target_position(n,t)
            velocity = self.target_velocity(n,t)
            origin=np.array((xyz[0],xyz[1],xyz[2]+self.vehicle_z_offsets[n]))
            p.resetBasePositionAndOrientation(body, origin, (0,0,0,1), physicsClientId=self.client)
            p.resetBaseVelocity(body, linearVelocity=velocity.tolist(), angularVelocity=(0,0,0), physicsClientId=self.client)
            for detail,offset,orientation in self.vehicle_details[n]:
                p.resetBasePositionAndOrientation(detail,origin+offset,orientation,physicsClientId=self.client)
                p.resetBaseVelocity(detail, linearVelocity=velocity.tolist(), angularVelocity=(0,0,0), physicsClientId=self.client)
            p.resetBasePositionAndOrientation(self.pad_ids[n], (xyz[0],xyz[1],xyz[2]+.035), (0,0,0,1), physicsClientId=self.client)
            p.resetBaseVelocity(self.pad_ids[n], linearVelocity=velocity.tolist(), angularVelocity=(0,0,0), physicsClientId=self.client)
            large,small=self.tag_plates[n]
            for body,point in ((large[0],(xyz[0]-.36,xyz[1],xyz[2]+.072)),(large[1],(xyz[0]-.36,xyz[1],xyz[2]+.072)),
                               (small[0],(xyz[0],xyz[1],xyz[2]+.076)),(small[1],(xyz[0],xyz[1],xyz[2]+.076))):
                p.resetBasePositionAndOrientation(body,point,(0,0,0,1),physicsClientId=self.client)
                p.resetBaseVelocity(body,linearVelocity=velocity.tolist(),angularVelocity=(0,0,0),physicsClientId=self.client)

