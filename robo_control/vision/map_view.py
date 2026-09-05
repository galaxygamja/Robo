"""Coordinate-map preview, independent of the camera's perspective view."""
import math
from .calibration import vision_dependencies


def draw_position_map(record, field_size_mm):
    cv2, np = vision_dependencies()
    width, height = field_size_mm
    scale = min(600/width, 600/height)
    canvas = np.full((680, 960, 3), (35,25,15), np.uint8)
    def point(x,y): return round(20+x*scale),round(30+(height-y)*scale)
    cv2.rectangle(canvas,point(0,height),point(width,0),(150,140,120),2)
    for obs in record.get("objects",[]):
        color={"red":(70,70,240),"green":(80,200,60),"yellow":(50,240,240)}.get(obs["color"],(220,220,220))
        cv2.circle(canvas,point(*obs["center_mm"]),6,color,-1)
    for i,track in enumerate(record.get("tracks",[])):
        rid,state=track["robot_id"],track["state"]
        coords=track.get("robot_center_mm")
        if coords is None:
            label=f"{rid}: {state}"
        else:
            x,y=coords;heading=track["heading_rad"]
            center=point(x,y)
            color=(130,230,80) if state=="observed" else (60,150,250)
            cv2.circle(canvas,center,15,color,2)
            cv2.arrowedLine(canvas,center,point(x+55*math.cos(heading),y+55*math.sin(heading)),color,2)
            cv2.putText(canvas,rid,(center[0]+18,center[1]),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
            label=f'{rid}: {x:.1f},{y:.1f} mm {math.degrees(heading):.1f}deg {state}'
        # All IDs remain in JSONL even if the display's finite legend is full.
        if i<25: cv2.putText(canvas,label,(625,35+i*23),cv2.FONT_HERSHEY_SIMPLEX,.4,(230,230,230),1)
    status="STOP REQUIRED" if record.get("stop_required",True) else "POSES OBSERVED - MOTORS DISCONNECTED"
    cv2.putText(canvas,status,(20,650),cv2.FONT_HERSHEY_SIMPLEX,.6,(60,180,255),2)
    return canvas
