"""Mouse-controlled waypoint editor for the Drone farm survey.

Use OPEN_MANUAL_PATH_EDITOR.bat on Windows. Save creates an atomic JSON file;
press U in the running Webots 3D viewport to load and fly the new route.
"""

import copy
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    raise SystemExit('Tkinter is missing. Install Python from python.org with Tcl/Tk support.')

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = ROOT / 'controllers' / 'drone'
sys.path.insert(0, str(CONTROLLER_DIR))
from manual_routes import MAX_WAYPOINTS, validate_manual_payload, check_safe_route
from planner import Planner

SCENE = json.loads((CONTROLLER_DIR / 'scene.json').read_text(encoding='utf-8'))
MANUAL_FILE = CONTROLLER_DIR / 'manual_mission.json'
BOUNDS = float(SCENE['bounds_m'])
COLORS = {
    'background':'#eef2eb','canvas':'#dcebd3', 'grid':'#b9d1b0',
    'corn':'#779c48', 'wheat':'#ddc375', 'lettuce':'#6caa75',
    'orchard':'#467b46', 'building':'#ba7968','water':'#5b9eb1',
    'route':'#167bd2','direct':'#6b7f6d','point':'#f8ae34',
    'selected':'#db4d35','home':'#197b5f', 'text':'#1b3525',
}


class ManualEditor:
    def __init__(self, root):
        self.root = root
        root.title('Drone | Path Editor')
        root.geometry('1250x850')
        root.minsize(1020,700)
        self.scene = SCENE
        self.stops = []
        self.selected = None
        self.dragging = False
        self.drag_start = None
        self.history = []
        self.legs = []
        self.route_length = None
        self.source_name = 'manual_mission.json'
        self.status = tk.StringVar(value='Click on the farm to add a waypoint')
        self.info = tk.StringVar(value='Route: not previewed')
        self.name = tk.StringVar()
        self.x = tk.StringVar()
        self.y = tk.StringVar()
        self.hover = tk.StringVar(value='4.0')
        self.take_photo = tk.BooleanVar(value=True)
        self.speed = tk.StringVar(value='3.8')
        self.altitude = tk.StringVar(value='13.0')
        self.clearance = tk.StringVar(value='1.2')
        self.preset = tk.StringVar(value='FULL_FARM_SURVEY')
        self.show_preview = tk.BooleanVar(value=True)
        self._build_gui()
        self.load_saved(silent=True)
        self.root.after(100, self.draw)

    def _build_gui(self):
        self.root.configure(bg='#e8efe7')
        header = tk.Frame(self.root,bg='#173b2f',height=75)
        header.pack(side=tk.TOP, fill=tk.X)
        tk.Label(header,text='WEBOTS   /   MANUAL FARM PATH EDITOR',font=('Segoe UI',19,'bold'),
                 fg='white',bg='#173b2f').pack(anchor=tk.W, padx=22,pady=(10,0))
        tk.Label(header,text='Click to add  |  Drag to move  |  Right-click to delete  |  Save then press U in Webots',
                 fg='#c7e8dc',bg='#173b2f',font=('Segoe UI',10)).pack(anchor=tk.W,padx=23)
        main=ttk.Frame(self.root,padding=10)
        main.pack(fill=tk.BOTH,expand=True)
        main.columnconfigure(0,weight=5)
        main.columnconfigure(1,weight=3)
        main.rowconfigure(0,weight=1)
        left=ttk.Frame(main)
        left.grid(row=0,column=0,sticky='nsew',padx=(0,10))
        left.rowconfigure(0,weight=1)
        left.columnconfigure(0,weight=1)
        self.canvas=tk.Canvas(left,bg=COLORS['canvas'],highlightthickness=0,cursor='crosshair')
        self.canvas.grid(row=0,column=0,sticky='nsew')
        self.canvas.bind('<Configure>', lambda event: self.draw())
        self.canvas.bind('<Button-1>',self.mouse_down)
        self.canvas.bind('<B1-Motion>',self.mouse_drag)
        self.canvas.bind('<ButtonRelease-1>',self.mouse_up)
        self.canvas.bind('<Button-3>',self.right_click)
        self.canvas.bind('<Double-Button-1>',lambda e: None)
        tk.Label(left,text='GREEN = Home    ORANGE = Waypoint    RED = Selected    BLUE = Safe A* preview',
                 bg='#edf2eb',fg='#476354',font=('Segoe UI',9)).grid(row=1,column=0,sticky='we',pady=(6,0))

        right=ttk.Frame(main,width=370,padding=8)
        right.grid(row=0,column=1,sticky='nsew')
        right.columnconfigure(0,weight=1)
        ttk.Label(right,text='Route waypoints',font=('Segoe UI',13,'bold')).grid(row=0,column=0,sticky='w')
        ttk.Label(right,text='Select a stop to edit or drag its numbered marker',wraplength=330).grid(row=1,column=0,sticky='w',pady=(3,7))
        list_frame=ttk.Frame(right)
        list_frame.grid(row=2,column=0,sticky='nsew')
        right.rowconfigure(2,weight=1)
        list_frame.rowconfigure(0,weight=1)
        list_frame.columnconfigure(0,weight=1)
        self.listbox=tk.Listbox(list_frame,font=('Consolas',10),height=10,selectmode=tk.SINGLE,
                                exportselection=False,activestyle='none')
        self.listbox.grid(row=0,column=0,sticky='nsew')
        scrollbar=ttk.Scrollbar(list_frame,orient='vertical',command=self.listbox.yview)
        scrollbar.grid(row=0,column=1,sticky='ns')
        self.listbox.config(yscrollcommand=scrollbar.set)
        self.listbox.bind('<<ListboxSelect>>',self.on_list_select)
        b_row=ttk.Frame(right)
        b_row.grid(row=3,column=0,sticky='ew',pady=(7,8))
        for i,(label,fn) in enumerate((('Up',self.up),('Down',self.down),('Delete',self.delete),('Undo',self.undo),('Clear',self.clear))):
            ttk.Button(b_row,text=label,command=fn).grid(row=0,column=i,padx=2,sticky='ew')
            b_row.columnconfigure(i,weight=1)

        details=ttk.LabelFrame(right,text=' Selected waypoint ',padding=6)
        details.grid(row=4,column=0,sticky='ew',pady=(0,5))
        details.columnconfigure(1,weight=1)
        details.columnconfigure(3,weight=1)
        ttk.Label(details,text='Name').grid(row=0,column=0,sticky='w',pady=1)
        ttk.Entry(details,textvariable=self.name).grid(row=0,column=1,columnspan=3,sticky='ew',padx=5,pady=1)
        ttk.Label(details,text='X').grid(row=1,column=0,sticky='w',pady=1)
        ttk.Entry(details,textvariable=self.x,width=10).grid(row=1,column=1,sticky='ew',padx=5,pady=1)
        ttk.Label(details,text='Y').grid(row=1,column=2,sticky='w',pady=1)
        ttk.Entry(details,textvariable=self.y,width=10).grid(row=1,column=3,sticky='ew',padx=5,pady=1)
        ttk.Label(details,text='Hover sec').grid(row=2,column=0,sticky='w',pady=1)
        ttk.Entry(details,textvariable=self.hover,width=9).grid(row=2,column=1,sticky='ew',padx=5,pady=1)
        ttk.Checkbutton(details,text='Capture photos',variable=self.take_photo).grid(row=2,column=2,columnspan=2,sticky='w',pady=1)
        ttk.Button(details,text='Apply selected changes',command=self.apply_selected).grid(row=3,column=0,columnspan=4,sticky='ew',pady=(4,0))

        flight=ttk.LabelFrame(right,text=' Flight parameters ',padding=6)
        flight.grid(row=5,column=0,sticky='ew',pady=(0,5))
        for j,(lab,var) in enumerate((('Speed (m/s)',self.speed),('Height (m)',self.altitude),('Clearance (m)',self.clearance))):
            flight.columnconfigure(j,weight=1)
            ttk.Label(flight,text=lab).grid(row=0,column=j,sticky='w',padx=(0,5))
            ttk.Entry(flight,textvariable=var,width=9).grid(row=1,column=j,sticky='ew',padx=(0,5))

        preset=ttk.Frame(right)
        preset.grid(row=6,column=0,sticky='ew',pady=(2,7))
        preset.columnconfigure(0,weight=1)
        ttk.Combobox(preset,textvariable=self.preset,values=list(SCENE['missions']),state='readonly').grid(row=0,column=0,sticky='ew',padx=(0,5))
        ttk.Button(preset,text='Load preset',command=self.load_preset).grid(row=0,column=1)
        actions=ttk.Frame(right)
        actions.grid(row=7,column=0,sticky='ew')
        actions.columnconfigure(0,weight=1)
        actions.columnconfigure(1,weight=1)
        ttk.Button(actions,text='Preview SAFE A* path',command=self.preview).grid(row=0,column=0,sticky='ew',padx=(0,5),pady=2)
        ttk.Button(actions,text='Reload saved file',command=self.load_saved).grid(row=0,column=1,sticky='ew',pady=2)
        style=ttk.Style()
        try: style.configure('Primary.TButton',font=('Segoe UI',11,'bold'))
        except tk.TclError: pass
        ttk.Button(actions,text='SAVE ROUTE FOR WEBOTS  (Ctrl+S)',command=self.save,style='Primary.TButton').grid(row=1,column=0,columnspan=2,sticky='ew',pady=(9,2))
        ttk.Label(right,textvariable=self.info,wraplength=350,foreground='#1b6d66').grid(row=8,column=0,sticky='w',pady=(5,1))
        ttk.Label(right,text='Webots: keep simulation running and press U in its 3D view to fly the saved manual route',
                  wraplength=350,foreground='#294f3c').grid(row=9,column=0,sticky='w',pady=(5,0))
        footer=tk.Frame(self.root,bg='#dce7d8',height=35)
        footer.pack(fill=tk.X)
        tk.Label(footer,textvariable=self.status,bg='#dce7d8',fg='#254c36',anchor='w',font=('Segoe UI',10)).pack(fill=tk.X,padx=15,pady=6)
        self.root.bind('<Control-s>',lambda event:self.save())
        self.root.bind('<Control-z>',lambda event:self.undo())
        self.root.bind('<Delete>',lambda event:self.delete() if self.root.focus_get()==self.listbox else None)

    def status_msg(self,text):
        self.status.set(text)

    def remember(self):
        self.history.append((copy.deepcopy(self.stops),self.selected))
        self.history=self.history[-30:]

    def invalidate(self):
        self.legs=[]
        self.route_length=None
        self.info.set('Route changed: click Preview to validate')

    def drawing_geometry(self):
        w=max(200,self.canvas.winfo_width()); h=max(200,self.canvas.winfo_height())
        side=min(w,h)-66
        ox=(w-side)/2; oy=(h-side)/2
        return ox,oy,side

    def world_to_screen(self,x,y):
        ox,oy,side=self.drawing_geometry()
        return ox+(x+BOUNDS)/(2*BOUNDS)*side, oy+(BOUNDS-y)/(2*BOUNDS)*side

    def screen_to_world(self,px,py):
        ox,oy,side=self.drawing_geometry()
        return (((px-ox)/side*2*BOUNDS-BOUNDS),
                (BOUNDS-(py-oy)/side*2*BOUNDS))

    def in_bounds(self,x,y):
        return -BOUNDS<=x<=BOUNDS and -BOUNDS<=y<=BOUNDS

    def draw_rect(self,cx,cy,sx,sy,**kw):
        x1,y1=self.world_to_screen(cx-sx/2,cy+sy/2)
        x2,y2=self.world_to_screen(cx+sx/2,cy-sy/2)
        return self.canvas.create_rectangle(x1,y1,x2,y2,**kw)

    def draw(self):
        c=self.canvas
        c.delete('all')
        ox,oy,side=self.drawing_geometry()
        c.create_rectangle(ox,oy,ox+side,oy+side,fill=COLORS['canvas'],outline='#719473',width=2)
        for val in range(-50,51,10):
            px,_=self.world_to_screen(val,0)
            _,py=self.world_to_screen(0,val)
            c.create_line(px,oy,px,oy+side,fill=COLORS['grid'],dash=(3,5))
            c.create_line(ox,py,ox+side,py,fill=COLORS['grid'],dash=(3,5))
            c.create_text(px,oy+side+12,text=str(val),font=('Segoe UI',8),fill='#6a8565')
            c.create_text(ox-17,py,text=str(val),font=('Segoe UI',8),fill='#6a8565')
        for fld in self.scene.get('fields',[]):
            col=COLORS.get(fld['kind'],'#86ad6b')
            self.draw_rect(fld['x'],fld['y'],fld['sx'],fld['sy'],fill=col,outline='#678c54',width=1)
            xx,yy=self.world_to_screen(fld['x'],fld['y'])
            c.create_text(xx,yy,text=fld['name'].split(' ')[0][:9],font=('Segoe UI',9,'bold'),fill='#233c27')
        for ob in self.scene.get('obstacles',[]):
            col=COLORS.get(ob.get('kind'),COLORS['building'])
            self.draw_rect(ob['x'],ob['y'],ob['sx'],ob['sy'],fill=col,outline='#80564b',width=2)
            xx,yy=self.world_to_screen(ob['x'],ob['y'])
            c.create_text(xx,yy,text=ob['name'].split(' ')[0][:9],font=('Segoe UI',8,'bold'),fill='white')
        if len(self.stops)>0:
            points=[self.scene['start']]+[s['point'] for s in self.stops]+[self.scene['start']]
            seg=[]
            for p in points:
                sx,sy=self.world_to_screen(*p)
                seg.extend([sx,sy])
            c.create_line(*seg,fill=COLORS['direct'],dash=(6,5),width=2)
        if self.legs:
            for leg in self.legs:
                seg=[]
                for p in leg:
                    px,py=self.world_to_screen(*p)
                    seg.extend([px,py])
                c.create_line(*seg,fill=COLORS['route'],width=3)
        hx,hy=self.world_to_screen(*self.scene['start'])
        c.create_oval(hx-11,hy-11,hx+11,hy+11,fill=COLORS['home'],outline='white',width=2)
        c.create_text(hx,hy,text='H',fill='white',font=('Segoe UI',9,'bold'))
        c.create_text(hx+16,hy-16,text='HOME',fill=COLORS['home'],font=('Segoe UI',9,'bold'),anchor='w')
        for i,s in enumerate(self.stops):
            px,py=self.world_to_screen(*s['point'])
            selected=i==self.selected
            radius=12 if selected else 10
            fill=COLORS['selected'] if selected else COLORS['point']
            c.create_oval(px-radius,py-radius,px+radius,py+radius,fill=fill,outline='white',width=2)
            c.create_text(px,py,text=str(i+1),fill='white' if selected else '#4f3922',font=('Segoe UI',9,'bold'))
            if selected:
                c.create_text(px,py-23,text=s['name'][:24],fill=COLORS['text'],font=('Segoe UI',9,'bold'))
        c.create_text(ox+10,oy+12,text='FARM MAP  /  metres  /  top view',fill='#395b42',anchor='w',font=('Segoe UI',10,'bold'))

    def nearest_index(self,px,py,threshold=17):
        match=None; best=threshold
        for i,stop in enumerate(self.stops):
            sx,sy=self.world_to_screen(*stop['point'])
            dist=math.hypot(px-sx,py-sy)
            if dist<best:
                match=i; best=dist
        return match

    def validate_point(self,x,y):
        if not self.in_bounds(x,y):
            return False,'Point is outside farm boundary'
        try:
            cfg=self.current_settings()
            planner=Planner(self.scene['obstacles'],boundary=BOUNDS,
                            resolution=.75,safety=cfg['obstacle_clearance_m'])
            if not planner.is_free(planner.xy_to_cell(x,y)):
                return False,'Too close to obstacle; choose another point'
        except ValueError as exc:
            return False,str(exc)
        return True,''

    def mouse_down(self,event):
        x,y=self.screen_to_world(event.x,event.y)
        index=self.nearest_index(event.x,event.y)
        if index is not None:
            self.set_selected(index)
            self.dragging=True
            self.drag_start=(index, copy.deepcopy(self.stops))
        elif self.in_bounds(x,y):
            if len(self.stops)>=MAX_WAYPOINTS:
                self.status_msg('Maximum %d points' % MAX_WAYPOINTS); return
            valid,reason=self.validate_point(x,y)
            if not valid:
                self.status_msg('Cannot add: '+reason); self.root.bell(); return
            self.remember()
            used={s['id'] for s in self.stops}
            n=1
            while 'CUSTOM_%02d'%n in used:n+=1
            stop={'id':'CUSTOM_%02d'%n,'name':'Custom Station %d'%n,
                  'point':[round(x,2),round(y,2)],'hover_s':4.,'take_photo':True}
            self.stops.append(stop)
            self.set_selected(len(self.stops)-1)
            self.invalidate()
            self.status_msg('Added station %d  (%.1f, %.1f)'%(len(self.stops),x,y))
        self.draw()

    def mouse_drag(self,event):
        if not self.dragging or self.selected is None:return
        x,y=self.screen_to_world(event.x,event.y)
        if not self.in_bounds(x,y):return
        # Drag should feel immediate: only check collision at release.
        self.stops[self.selected]['point']=[round(x,2),round(y,2)]
        self.invalidate()
        self.draw()

    def mouse_up(self,event):
        if not self.dragging:return
        self.dragging=False
        if self.drag_start is None:return
        idx,prior=self.drag_start
        self.drag_start=None
        candidate=self.stops[idx]['point']
        valid,reason=self.validate_point(*candidate)
        if not valid:
            self.stops=prior
            self.status_msg('Invalid move cancelled: '+reason)
            self.root.bell()
        elif prior != self.stops:
            self.history.append((prior,idx))
            self.history=self.history[-30:]
            self.status_msg('Moved stop %d. Press Preview then Save' % (idx+1))
        self.refresh_list()
        self.draw()

    def right_click(self,event):
        i=self.nearest_index(event.x,event.y,22)
        if i is not None:
            self.set_selected(i)
            self.delete()

    def set_selected(self,index):
        if index is None or index>=len(self.stops):
            self.selected=None
            self.refresh_list()
            self.draw()
            return
        self.selected=index
        self.refresh_list()
        stop=self.stops[index]
        self.name.set(stop['name'])
        self.x.set(str(stop['point'][0]))
        self.y.set(str(stop['point'][1]))
        self.hover.set(str(stop.get('hover_s',4.)))
        self.take_photo.set(stop.get('take_photo',True))
        self.draw()

    def refresh_list(self):
        self.listbox.delete(0,tk.END)
        for i,s in enumerate(self.stops):
            flag='CAM' if s['take_photo'] else '---'
            self.listbox.insert(tk.END,f"{i+1:02d}  {s['name'][:20]:20}  {flag}")
        if self.selected is not None and self.selected<len(self.stops):
            self.listbox.selection_set(self.selected)
            self.listbox.see(self.selected)

    def on_list_select(self,event):
        sel=self.listbox.curselection()
        if sel:
            index=sel[0]
            if index!=self.selected:
                self.selected=index
                stop=self.stops[index]
                self.name.set(stop['name'])
                self.x.set(str(stop['point'][0]))
                self.y.set(str(stop['point'][1]))
                self.hover.set(str(stop.get('hover_s',4.)))
                self.take_photo.set(stop.get('take_photo',True))
                self.draw()

    def apply_selected(self):
        if self.selected is None:
            self.status_msg('Select a waypoint first');return
        try:
            name=self.name.get().strip()
            x=float(self.x.get());y=float(self.y.get());hover=float(self.hover.get())
            if not name or len(name)>64:raise ValueError('Name must be 1-64 characters')
            if not math.isfinite(hover) or not 1.2<=hover<=20:
                raise ValueError('Hover must be between 1.2 and 20 sec')
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('Coordinates must be numbers')
            valid,reason=self.validate_point(x,y)
            if not valid:raise ValueError(reason)
            self.remember()
            self.stops[self.selected].update(name=name,point=[round(x,2),round(y,2)],
                                             hover_s=round(hover,2),take_photo=self.take_photo.get())
            self.invalidate(); self.refresh_list();self.draw()
            self.status_msg('Updated selected waypoint')
        except ValueError as exc:
            messagebox.showerror('Invalid waypoint',str(exc))

    def up(self):self.move_selected(-1)
    def down(self):self.move_selected(1)

    def move_selected(self,delta):
        if self.selected is None:return
        new=self.selected+delta
        if new<0 or new>=len(self.stops):return
        self.remember()
        self.stops[self.selected],self.stops[new]=self.stops[new],self.stops[self.selected]
        self.selected=new;self.invalidate();self.set_selected(new)

    def delete(self):
        if self.selected is None:return
        self.remember()
        self.stops.pop(self.selected)
        self.invalidate()
        self.selected=min(self.selected,len(self.stops)-1) if self.stops else None
        self.set_selected(self.selected)
        self.status_msg('Waypoint deleted')

    def clear(self):
        if self.stops and messagebox.askyesno('Clear route','Remove all waypoints from editor?'):
            self.remember();self.stops=[];self.selected=None
            self.invalidate();self.refresh_list();self.draw()
            self.status_msg('Route cleared (not saved yet)')

    def undo(self):
        if not self.history:
            self.status_msg('Nothing to undo');return
        self.stops,self.selected=self.history.pop()
        self.invalidate();self.set_selected(self.selected)
        self.status_msg('Undo complete')

    def load_preset(self):
        key=self.preset.get()
        scene_ids={s['id']:s for s in self.scene['photo_stops']}
        if key not in self.scene['missions']:return
        self.remember()
        self.stops=[]
        for i,sid in enumerate(self.scene['missions'][key],1):
            s=scene_ids[sid]
            self.stops.append({'id':'CUSTOM_%02d'%i,'name':s['name'],
                               'point':s['point'][:],'hover_s':4.,'take_photo':True})
        self.invalidate();self.set_selected(0)
        self.status_msg('%s loaded as an editable manual copy (%d stops)'%(key,len(self.stops)))

    def current_settings(self):
        def get(var,label,low,high):
            try:v=float(var.get())
            except ValueError:raise ValueError(label+' must be numeric')
            if not math.isfinite(v) or v<low or v>high:
                raise ValueError('%s needs value from %g to %g'%(label,low,high))
            return round(v,2)
        return {'flight_speed_mps':get(self.speed,'Speed',.5,7),
                'target_altitude_m':get(self.altitude,'Altitude',6,24),
                'obstacle_clearance_m':get(self.clearance,'Clearance',.4,2.5)}

    def compose_payload(self):
        payload={'version':1,'name':'Student Manual Farm Mission',
                 'stops':copy.deepcopy(self.stops),'flight_settings':self.current_settings()}
        stops,config=validate_manual_payload(payload,self.scene)
        return payload,stops,config

    def preview(self):
        try:
            payload,stops,config=self.compose_payload()
            self.status_msg('Calculating A* clearance-safe route...')
            self.root.update_idletasks()
            legs,distance=check_safe_route(stops,self.scene,config,resolution=.75)
            self.legs=legs
            self.route_length=distance
            photo_count=sum(int(s['take_photo']) for s in stops)
            eta=(distance/config['flight_speed_mps'] + sum(s['hover_s'] for s in stops))
            self.info.set('%d stops  |  %.1f m  |  about %.1f min  |  %d photo stops'%(len(stops),distance,eta/60,photo_count))
            self.status_msg('SAFE A* route is valid. Blue line is planned path; dashed line is direct route.')
            self.draw()
            return True
        except ValueError as exc:
            self.legs=[];self.route_length=None;self.draw()
            self.info.set('Route cannot be saved until errors are fixed')
            self.status_msg('ROUTE ERROR: '+str(exc))
            messagebox.showerror('Route validation failed',str(exc))
            return False

    def save(self):
        if not self.preview():return
        payload,_,_=self.compose_payload()
        payload['saved_at']=datetime.now().isoformat(timespec='seconds')
        temp=MANUAL_FILE.with_suffix('.json.tmp')
        try:
            temp.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            os.replace(temp,MANUAL_FILE)
        except OSError as exc:
            messagebox.showerror('Cannot save route',str(exc));return
        self.status_msg('SAVED! Now focus the Webots 3D view and press U to start your NEW manual path')
        messagebox.showinfo('Route saved for Webots',
            'Manual mission saved successfully!\n\n'
            '1  Leave Webots simulation running\n'
            '2  Click its 3D view to focus it\n'
            '3  Press U to load and fly these points\n\n'
            'Check Webots Console for [MANUAL ROUTE] log lines')

    def load_saved(self,silent=False):
        try:
            payload=json.loads(MANUAL_FILE.read_text(encoding='utf-8'))
            stops,config=validate_manual_payload(payload,self.scene)
            self.stops=stops
            self.history=[]
            self.speed.set(str(config['flight_speed_mps']))
            self.altitude.set(str(config['target_altitude_m']))
            self.clearance.set(str(config['obstacle_clearance_m']))
            self.invalidate();self.set_selected(0)
            self.status_msg('Loaded saved route (%d points). Edit, Preview, Save, then press U in Webots.'%len(stops))
        except (OSError,ValueError,json.JSONDecodeError) as exc:
            if not silent:messagebox.showerror('Cannot load manual route',str(exc))
            else:self.status_msg('No valid saved route found: '+str(exc))


def main():
    root=tk.Tk()
    try:
        style=ttk.Style(root)
        if 'vista' in style.theme_names():style.theme_use('vista')
        elif 'clam' in style.theme_names():style.theme_use('clam')
    except tk.TclError:pass
    ManualEditor(root)
    root.mainloop()


if __name__=='__main__':main()
