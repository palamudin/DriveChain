/* === physics.js === */
/* Tarmac Lab vehicle solver. Right-handed, Y-up, SI units. Car forward is +Z; driver-left is +X.
 * Generalised coordinates u = [v (3), ω (3), ṡ (4)]: sprung body plus one travel DOF per corner.
 * Unsprung parts are point masses whose velocity is v + ω×r + Σ e_k ṡ_k, e_k from the suspension
 * Jacobians. M(q) is assembled every contact substep and inverted (Cholesky, 10×10). Momentum form:
 * after the positions move, u is re-mapped so the generalised momentum M·u is carried across the
 * configuration change (linear momentum exactly; angular momentum about the moving body point gets
 * its −v×P term); travel DOF get Lagrange's ∂T/∂s at fixed velocities, with the linkage curvature
 * ∂e/∂s by central differences. Energy is then conserved to the integrator's order. All forces
 * enter as Jᵀf, so link geometry, jacking, roll centres and anti-dive/anti-squat come from
 * virtual work, never from coefficients. Wheel spin ω_w is relative to its carrier: outboard brake
 * torque is internal to the corner; drive torque from a body-mounted differential also works
 * on the carrier's side-view pitch (anti-squat of independent suspensions).
 * No upright assist, yaw controller or velocity steering.
 */
(function(root){
  'use strict';
  const {V,Q,clamp,approach}=root.TM;
  const VERSION='0.5.0-physics-merge';
  // Node callers that load the older module list still get explicit feature dependencies.
  if(typeof require==='function'){if(!root.TarmacThermal)require('./thermal.js');if(!root.TarmacMaterials)require('./materials.js');if(!root.TarmacDifferential)require('./differential.js');}
  const PT=root.TarmacPowertrain,SUS=root.TarmacSuspension,TY=root.TarmacTyre,SURF=root.TarmacSurface,DMG=root.TarmacDamage,ENV=root.TarmacEnvironment,TH=root.TarmacThermal,MAT=root.TarmacMaterials,DIFF=root.TarmacDifferential;
  const PROFILES=Object.freeze({legacy:Object.freeze({physicsProfile:'legacy',thermal:{enabled:false},liftFront:0,liftRear:0}),enhanced:Object.freeze({physicsProfile:'enhanced',thermal:{enabled:true},liftFront:.10,liftRear:.14})});
  const DEFAULTS=Object.freeze({
    physicsProfile:'legacy',thermal:{...TH.THERMAL_DEFAULTS},diff:{...DIFF.DEFAULTS},liftFront:0,liftRear:0,aeroHeight:.62,throttleRate:3,brakeRate:5,
    mass:1200, wheelbase:2.6, track:1.6, cgHeight:0.62, wheelInertia:1.35,
    restLength:0.46, minLength:0.30, bumpRate:220000,
    mu:1.0, loadSensitivity:0.08, brushKx:130000, brushKy:110000,
    brushDamping:420, relaxationX:0.35, relaxationY:0.40,
    powertrain:PT.clone(PT.DEFAULTS), brakeTorque:4700, frontBrakeBias:0.62,
    handbrakeTorque:2800, rollingResistance:0.012, dragArea:0.65,
    steerMax:28, steerRate:65, drive:'RWD', gravity:9.81,
    hz:240, substeps:2, iterations:8,
    suspension:{front:{...SUS.AXLE_DEFAULTS.front},rear:{...SUS.AXLE_DEFAULTS.rear}},
    tyre:{...TY.TYRE_DEFAULTS}, damage:{...DMG.DAMAGE_DEFAULTS}, surface:'provingGround',environment:{...ENV.DEFAULTS},
  });
  const BOUNDS={liftFront:[-1.5,1],liftRear:[-1.5,1],aeroHeight:[.2,1.2],throttleRate:[2,60],brakeRate:[2,60],
    mass:[500,2500],wheelbase:[2,3.4],track:[1.3,2.1],cgHeight:[.35,.95],
    wheelInertia:[.6,3.5],mu:[.15,1.6],
    brakeTorque:[1200,8000],steerMax:[10,45],steerRate:[20,140],
    restLength:[.38,.55],minLength:[.08,.36],bumpRate:[100000,600000],
    loadSensitivity:[0,.2],brushKx:[30000,250000],brushKy:[30000,250000],
    brushDamping:[50,1800],relaxationX:[.1,1.0],relaxationY:[.1,1.0],
    frontBrakeBias:[.4,.8],handbrakeTorque:[0,6000],rollingResistance:[0,.035],
    dragArea:[0,1.5],gravity:[1,20],hz:[120,480],substeps:[1,4],iterations:[4,16],
  };
  const NESTED=['powertrain','suspension','tyre','damage','surface','environment','physicsProfile','thermal','diff'];
  function validateConfig(raw){
    if(!raw || typeof raw!=='object' || Array.isArray(raw))throw new Error('Configuration must be an object.');
    const profile=raw.physicsProfile??'legacy';if(!Object.hasOwn(PROFILES,profile))throw Error('Unknown physics profile');
    const out={...DEFAULTS,...PROFILES[profile],powertrain:PT.validate(raw.powertrain||{})};
    if(raw.thermal!==undefined&&(!raw.thermal||typeof raw.thermal!=='object'||Array.isArray(raw.thermal)))throw Error('thermal must be an object');out.thermal=TH.validateThermal({...PROFILES[profile].thermal,...raw.thermal});out.diff=DIFF.validate(raw.diff);
    for(const key of Object.keys(DEFAULTS)){
      if(!(key in raw)||NESTED.includes(key))continue;
      if(key==='drive'){if(!['RWD','FWD','AWD'].includes(raw[key]))throw new Error('Drive must be RWD, FWD or AWD.');out[key]=raw[key];continue;}
      const x=Number(raw[key]);if(!Number.isFinite(x))throw new Error(`${key} must be a finite number.`);
      if(BOUNDS[key]&&(x<BOUNDS[key][0]||x>BOUNDS[key][1]))throw new Error(`${key} must be in ${BOUNDS[key].join(' … ')}.`);
      out[key]=x;
    }
    // 0.2 configurations carried one spring / damper for all corners: seed both axles.
    const legacy={};if('spring' in raw)legacy.springRate=Number(raw.spring);if('damper' in raw)legacy.damperRate=Number(raw.damper);
    const sus=raw.suspension||{};if(typeof sus!=='object'||Array.isArray(sus))throw new Error('suspension must be an object.');
    const seed=w=>sus[w]??(Object.keys(legacy).length?{...DEFAULTS.suspension[w],...legacy}:null);
    out.suspension={front:SUS.validateAxle(seed('front'),'front'),rear:SUS.validateAxle(seed('rear'),'rear')};
    out.tyre=TY.validateTyre(raw.tyre);out.damage=DMG.validateDamage(raw.damage);
    if('surface' in raw){
      const surface=raw.surface;
      if(surface&&typeof surface==='object'&&surface.kind==='city'){
        if(!root.TarmacCitySurface)throw new Error('City collision adapter is not available.');
        out.surface=root.TarmacCitySurface.descriptor(surface);
      }else{
        if(typeof surface==='string'&&!['provingGround','flat','world'].includes(surface))throw new Error('Unknown surface.');
        if(typeof surface!=='string'&&(!surface||typeof surface!=='object'||Array.isArray(surface)||surface.kind!=='world'))throw new Error('Invalid surface descriptor.');
        if(typeof surface==='object'&&(('version' in surface&&surface.version!==1)||('tileSize' in surface&&surface.tileSize!==1000)||('seed' in surface&&(!Number.isInteger(surface.seed)||surface.seed<0||surface.seed>4294967295))||('tiles' in surface&&(!Array.isArray(surface.tiles)||surface.tiles.length!==16||!surface.tiles.every(x=>Number.isFinite(x)&&x>=0&&x<=1)))))throw new Error('Invalid world descriptor version, seed or tiles.');
        out.surface=surface==='world'||typeof surface==='object'?SURF.worldDescriptor(typeof surface==='object'?surface:{}):surface;
      }
    }
    if(!('aeroHeight'in raw))out.aeroHeight=out.cgHeight;
    out.environment=ENV.validate(raw.environment);
    if(![120,240,480].includes(out.hz))throw new Error('Physics frequency must be 120, 240 or 480 Hz.');
    if(!Number.isInteger(out.substeps)||!Number.isInteger(out.iterations))throw new Error('Solver counts must be integers.');
    if(out.minLength>out.restLength-.05)throw new Error('minLength must be at least 50 mm shorter than restLength.');
    out.radius=TY.parseSize(out.tyre.size).R0; // derived from the size code; informational
    return out;
  }
  // New-version replay snapshots must carry resolved canonical configuration.
  // Config imports and historical migrations keep their intentional defaults.
  function requireResolved(value,reference,label){
    if(reference&&typeof reference==='object'){
      if(!value||typeof value!=='object'||Array.isArray(value)!==Array.isArray(reference))throw Error('Missing resolved snapshot '+label);
      if(Array.isArray(reference)&&value.length!==reference.length)throw Error('Invalid resolved snapshot '+label);
      for(const key of Object.keys(reference)){if(!Object.prototype.hasOwnProperty.call(value,key))throw Error('Missing resolved snapshot '+label+'.'+key);requireResolved(value[key],reference[key],label+'.'+key);}
    }else if(typeof value!==typeof reference||value!==reference)throw Error('Noncanonical snapshot '+label);
  }
  const emptyInput=()=>({throttle:0,brake:0,steer:0,handbrake:0,gear:1,clutch:0,starter:0});
  function cleanInput(i={}){return {throttle:clamp(+i.throttle||0,0,1),brake:clamp(+i.brake||0,0,1),steer:clamp(+i.steer||0,-1,1),handbrake:clamp(+i.handbrake||0,0,1),gear:Number.isInteger(i.gear)&&i.gear>=-1&&i.gear<=6?i.gear:1,clutch:clamp(+i.clutch||0,0,1),starter:i.starter?1:0};}
  const N=10;
  /* Cholesky inverse of a symmetric positive-definite 10×10 (row-major). Deterministic. */
  function invertSPD(A,L,out){
    L.fill(0);
    for(let i=0;i<N;i++)for(let j=0;j<=i;j++){let s=A[i*N+j];for(let k=0;k<j;k++)s-=L[i*N+k]*L[j*N+k];
      if(i===j){if(s<=0)throw new Error('Mass matrix is not positive definite.');L[i*N+i]=Math.sqrt(s);}else L[i*N+j]=s/L[j*N+j];}
    const Li=out.Li||(out.Li=new Float64Array(N*N));Li.fill(0);
    for(let i=0;i<N;i++){Li[i*N+i]=1/L[i*N+i];for(let j=0;j<i;j++){let s=0;for(let k=j;k<i;k++)s-=L[i*N+k]*Li[k*N+j];Li[i*N+j]=s/L[i*N+i];}}
    for(let i=0;i<N;i++)for(let j=0;j<=i;j++){let s=0;for(let k=i;k<N;k++)s+=Li[k*N+i]*Li[k*N+j];out[i*N+j]=s;out[j*N+i]=s;}
  }
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const dot10=(a,b)=>{let s=0;for(let i=0;i<N;i++)s+=a[i]*b[i];return s;};
  const SLIDE_FLOOR=.7,SLIDE_SPEED=5; // μ_slide/μ_peak asymptote; characteristic excess sliding speed (m/s)
  const SCENARIO_START={surface:{x:0,z:-45,yaw:Math.PI},curb:{x:0,z:-252,yaw:Math.PI}};

  class Vehicle {
    constructor(config={}){this.cfg=validateConfig(config);this.build();this.reset();}
    build(){
      const c=this.cfg;
      this.surface=c.surface?.kind==='city'?root.TarmacCitySurface.resolve(c.surface):new SURF.Surface(c.surface);this.tyre=new TY.Tyre(c.tyre);
      const g=c.gravity,unsprungOf=w=>{const sp=c.suspension[w],t=SUS.TYPES[sp.type],m=sp.unsprungMass||t.unsprung;return t.kind==='solid'?m:2*m;};
      this.unsprung=[unsprungOf('front'),unsprungOf('rear')];this.mS=c.mass-this.unsprung[0]-this.unsprung[1];
      if(this.mS<c.mass*.6)throw new Error('Unsprung mass exceeds 40 % of the total mass.');
      this.cornerLoad=c.mass*g/4;this.cornerSprung=this.mS*g/4;
      const Rl=this.tyre.R0-this.cornerLoad/this.tyre.Kz;
      this.axles=['front','rear'].map(w=>new SUS.Axle(c.suspension[w],w,{track:c.track,wheelbase:c.wheelbase,cgHeight:c.cgHeight,restLength:c.restLength,minLength:c.minLength,loadedRadius:Rl,designTravel:this.cornerSprung/c.suspension[w].springRate}));
      this.inertia=[this.mS/12*(4.3**2+1.1**2),this.mS/12*(4.3**2+1.8**2),this.mS/12*(1.8**2+1.1**2)];
      this.M=new Float64Array(N*N);this.Minv=new Float64Array(N*N);this.Lc=new Float64Array(N*N);this.F=new Float64Array(N);this.tmp=new Float64Array(N);
      this.static=this.axles.map(a=>a.staticTravel(this.cornerSprung,{bumpRate:c.bumpRate}));this.axlePose=[null,null];
      // Reference footprint (static corner load, flat road, zero camber) for load-dependent slip stiffness.
      {const t=this.tyre,d=this.cornerLoad/t.Kz,st=t.makeState();t.contact({c:[0,t.R0-d,0],a:[1,0,0],d:[0,-1,0],f:[0,0,1],s:[-1,0,0],shiftY:0,shiftX:0,roll:0,dent:null,spin:0,Kz:t.Kz,pressure:c.tyre.pressure},new SURF.Surface('flat'),st);
       this.patch0=st.areaSmooth*st.halfLength;
       if(!Number.isFinite(this.patch0)||this.patch0<=0)throw new Error('Reference tyre footprint must have finite positive area and pressure moment.');}
    }
    attachSurface(surface,{reset=true,spawn}={}){
      if(!surface||typeof surface.raycast!=='function')throw new Error('Invalid surface adapter.');
      if(surface.kind==='city'){
        const id=surface.descriptor?.id||'city-'+(surface.manifest.source?.sha256||surface.manifest.name||'local');
        this.cfg.surface=root.TarmacCitySurface.register(id,surface);this.surface=surface.createContext();this.surface.descriptor={...this.cfg.surface};
      }else{this.surface=surface;this.cfg.surface=surface.descriptor||surface.kind;}
      if(this.environment)this.surface.environment=this.environment;
      if(reset)this.reset(spawn);return this;
    }
    reset(options={}){
      const c=this.cfg,city=this.surface.kind==='city',spawn=city?this.surface.manifest.spawn:null;
      const {drop=0,x=spawn?.position?.[0]??0,z=spawn?.position?.[2]??0,yaw=spawn?.yaw??0,originY}=options;
      if(city)this.surface.assertReady(x-4,x+4,z-4,z+4);
      const select=city&&Number.isFinite(originY)?{originY}:city&&options.x===undefined&&options.z===undefined&&spawn?.position?{originY:spawn.position[1]+1}:undefined;
      const spawnHeight=this.surface.heightAt(x,z,undefined,select);
      if(!Number.isFinite(spawnHeight))throw new Error('There is no loaded driveable surface at the requested spawn.');
      this.damage=new DMG.Damage(c.damage,c.tyre.pressure,this.cornerLoad);
      this.environment=new ENV.Environment(c.environment);this.surface.environment=this.environment;this.env=this.environment;
      this.thermal=new TH.Thermal(c.thermal,this.tyre,c.tyre.pressure);this.thermal.reset(this.environment);this.diffHeatJ=0;this.diffProjectionJ=0;this.diffTorque=[];this.aero={drag:[0,0,0],lift:[0,0],airspeed:0};
      this.s=[this.static[0],this.static[0],this.static[1],this.static[1]];this.sd=[0,0,0,0];
      const P=this.axles.map((a,k)=>a.pose(this.s[2*k],this.s[2*k+1]));
      const wcY=(P[0][0].wc[1]+P[1][0].wc[1])/2,deflection=this.cornerLoad/this.tyre.Kz;
      const equilibrium=spawnHeight+this.tyre.R0-deflection-wcY;
      this.p=[x,equilibrium+0.06+drop,z];this.v=[0,0,0];this.q=Q.axis([0,1,0],yaw);this.w=[0,0,0];
      this.tick=0;this.time=0;this.steer=0;this.throttle=0;this.brake=0;this.input=emptyInput();
      this.accel=[0,0,0];this.travel=0;this.bodyContacts=0;this.invalid=false;this.boundary=false;this.loading=false;
      this.wheels=[['FL',1,1],['FR',-1,1],['RL',1,-1],['RR',-1,-1]].map(([id,sx,sz],k)=>({
        id,sx,sz,index:k,axle:sz>0?0:1,mount:[sx*c.track/2,.70-c.cgHeight,sz*c.wheelbase/2],
        omega:0,angle:0,brush:[0,0],length:c.restLength-this.s[k],compression:this.s[k],normalLoad:0,
        fx:0,fy:0,slip:0,slipAngle:0,usage:0,mu:c.mu,contact:false,steer:0,toe:0,camber:0,
        origin:[0,0,0],point:[0,0,0],center:[0,0,0],normal:[0,1,0],force:[0,0,0],r:[0,0,0],
        patch:this.tyre.makeState(),Kz:this.tyre.Kz,Re:this.tyre.R0,susp:{spring:0,damper:0,damperSpeed:0,bumpStop:0},
        slidePower:0,rimForce:0,camberSlip:0,wcBody:[0,0,0],eB:[[0,0,0],[0,0,0],[0,0,0],[0,0,0]],WB:[[0,0,0],[0,0,0],[0,0,0],[0,0,0]],
      }));
      this.powertrain=new PT.Powertrain(c.powertrain);this.massValid=false;
      this.refreshVisualContacts();this.previous=this.pose();
      return this;
    }
    pose(){const W=this.wheels||[];return {p:this.p.slice(),q:this.q.slice(),wheelAngles:W.map(x=>x.angle),lengths:W.map(x=>x.length),steers:W.map(x=>x.steer),
      centers:W.map(x=>x.wcBody.slice()),yaws:W.map(x=>x.steer-x.sx*x.toe),rolls:W.map(x=>-x.sx*x.camber)};}
    /* ---- Kinematics: suspension pose + Jacobians in body frame, then world frames. ---- */
    kinematics(){
      for(let a=0;a<2;a++){
        const P=this.axles[a].evaluate(this.s[2*a],this.s[2*a+1]);this.axlePose[a]=P;
        for(let i=0;i<2;i++){const k=2*a+i,w=this.wheels[k],eff=this.damage.effects(k);
          w.wcBody=P[i].wc;for(let j=0;j<4;j++){w.eB[j]=j===2*a?P[i].e[0]:j===2*a+1?P[i].e[1]:[0,0,0];w.WB[j]=j===2*a?P[i].W[0]:j===2*a+1?P[i].W[1]:[0,0,0];}
          w.camber=P[i].camber+eff.camber;w.toe=P[i].toe+eff.toe;w.eff=eff;}
      }
      for(const w of this.wheels){
        w.center=V.add(this.p,Q.rot(this.q,w.wcBody));w.origin=V.add(this.p,Q.rot(this.q,w.mount));
        w.qw=Q.mul(this.q,Q.mul(Q.axis([0,1,0],w.steer-w.sx*w.toe),Q.axis([0,0,1],-w.sx*w.camber)));
        w.axis=Q.rot(w.qw,[1,0,0]);w.down=Q.rot(w.qw,[0,-1,0]);w.plane=Q.rot(w.qw,[0,0,1]);
        w.eW=w.eB.map(e=>Q.rot(this.q,e));w.WW=w.WB.map(e=>Q.rot(this.q,e));
        w.length=this.cfg.restLength-this.s[w.index];w.compression=this.s[w.index];
      }
    }
    /* Jacobian columns of a world point P on wheel w's carrier: only its axle's two travel DOF are non-zero. */
    pointE(w,P,j){if(j>>1!==w.axle)return null;const e=w.eW[j],W=w.WW[j],dx=P[0]-w.center[0],dy=P[1]-w.center[1],dz=P[2]-w.center[2];
      return [e[0]+W[1]*dz-W[2]*dy,e[1]+W[2]*dx-W[0]*dz,e[2]+W[0]*dy-W[1]*dx];}
    genVec(w,P,dir,out){const rx=P[0]-this.p[0],ry=P[1]-this.p[1],rz=P[2]-this.p[2];out[0]=dir[0];out[1]=dir[1];out[2]=dir[2];
      out[3]=ry*dir[2]-rz*dir[1];out[4]=rz*dir[0]-rx*dir[2];out[5]=rx*dir[1]-ry*dir[0];out[6]=out[7]=out[8]=out[9]=0;
      for(let j=2*w.axle;j<2*w.axle+2;j++){const e=this.pointE(w,P,j);out[6+j]=e[0]*dir[0]+e[1]*dir[1]+e[2]*dir[2];}return out;}
    pointVelocityOn(w,P){const rx=P[0]-this.p[0],ry=P[1]-this.p[1],rz=P[2]-this.p[2],v=this.v,o=this.w;
      let x=v[0]+o[1]*rz-o[2]*ry,y=v[1]+o[2]*rx-o[0]*rz,z=v[2]+o[0]*ry-o[1]*rx;
      for(let j=2*w.axle;j<2*w.axle+2;j++){const e=this.pointE(w,P,j),s=this.sd[j];x+=e[0]*s;y+=e[1]*s;z+=e[2]*s;}return [x,y,z];}
    pointVelocity(r){return V.add(this.v,V.cross(this.w,r));}
    contactWheel(w){
      const dm=this.damage.corners[w.index],sl=Math.hypot(w.axis[0],w.axis[2])||1;
      w.pressure=this.thermal.pressure(w.index,dm.pressure,this.environment.p);w.Kz=this.tyre.kz(w.pressure);
      this.tyre.contact({c:w.center,a:w.axis,d:w.down,f:w.plane,s:[-w.axis[0]/sl,0,-w.axis[2]/sl],shiftY:w.brush[1],shiftX:w.brush[0],
        roll:this.cfg.tyre.carcassRoll*w.brush[1]/this.tyre.H,dent:dm.dent>0?{depth:dm.dent,angle:dm.dentAngle,width:.25}:null,spin:w.angle,Kz:w.Kz,pressure:w.pressure},this.surface,w.patch);
      const st=w.patch;w.contact=st.contact;w.point=st.contact?st.Pc.slice():V.add(w.center,V.mul(w.down,this.tyre.R0));w.normal=st.contact?st.n.slice():[0,1,0];
      w.Re=this.tyre.R0-st.deltaEff/3; // effective rolling radius of a radial tyre
    }
    surfaceMix(w){
      const sample=this.surface.sampleAt?this.surface.sampleAt(w.point[0],w.point[2],{},this.surface.kind==='city'?{originY:w.point[1]+.04,maxDistance:.12}:undefined):{};
      w.road=this.environment.conditions(sample);w.material=sample.material||'asphalt';w.water=0;w.hydro=0;w.surfaceMu=w.road.frictionMultiplier;
      if(this.cfg.physicsProfile==='enhanced'&&w.contact){w.mix=MAT.mix(w.patch,this.surface);w.material=w.mix.material;}
    }
    surfaceGrip(w,speed){
      if(this.cfg.physicsProfile!=='enhanced')return;
      const r=MAT.grip(w.mix,speed,w.pressure,this.environment.cfg.rainMm,this.environment.surfaceTemperature,w.road.wetness);
      w.water=r.water;w.hydro=r.hydro;w.surfaceMu=r.mu*(1-.64*w.road.snow)*(1-.80*w.road.ice);
      w.road.frictionMultiplier=w.surfaceMu;w.road.rollingResistanceMultiplier=w.mix.rr*(1+.12*w.road.wetness+1.8*w.road.snow+.1*w.road.ice);
      w.road.slideFloor=clamp(w.mix.slideFloor-.05*w.road.wetness-.07*w.road.ice,.5,1.05);w.slideRef=w.mix.slideSpeed;
    }
    refreshVisualContacts(){this.kinematics();for(const w of this.wheels)this.contactWheel(w);}
    /* ---- Mass matrix: sprung body + unsprung point masses. ---- */
    buildMass(){
      const M=this.M;M.fill(0);const m=this.mS;M[0]=M[11]=M[22]=m;
      const ax=[Q.rot(this.q,[1,0,0]),Q.rot(this.q,[0,1,0]),Q.rot(this.q,[0,0,1])];
      for(let a=0;a<3;a++)for(let b=0;b<3;b++){let s=0;for(let k=0;k<3;k++)s+=this.inertia[k]*ax[k][a]*ax[k][b];M[(3+a)*N+3+b]=s;}
      const J=this.J||(this.J=[new Float64Array(N),new Float64Array(N),new Float64Array(N)]);
      for(let a=0;a<2;a++)for(const pm of this.axles[a].masses(this.axlePose[a])){
        const w=this.wheels[2*a+pm.side],P=V.add(w.center,Q.rot(this.q,pm.offset)),r=V.sub(P,this.p);
        for(let row=0;row<3;row++){J[row].fill(0);J[row][row]=1;}
        J[0][4]=r[2];J[0][5]=-r[1];J[1][3]=-r[2];J[1][5]=r[0];J[2][3]=r[1];J[2][4]=-r[0]; // ω×r = −[r]× ω
        for(let j=0;j<4;j++){const e=this.pointE(w,P,j);if(e)for(let row=0;row<3;row++)J[row][6+j]=e[row];}
        for(let row=0;row<3;row++)for(let i=0;i<N;i++){const ji=J[row][i];if(!ji)continue;for(let k=0;k<N;k++)M[i*N+k]+=pm.mass*ji*J[row][k];}
      }
      invertSPD(M,this.Lc,this.Minv);
    }
    /* World Jacobians of every unsprung point mass of axle a at travel (sL, sR), orientation fixed. */
    massJacobians(a,sL,sR){
      const P=this.axles[a].evaluate(sL,sR),out=[];
      for(const pm of this.axles[a].masses(P)){const side=P[pm.side],E=[0,1].map(j=>{const W=side.W[j],o=pm.offset;return Q.rot(this.q,V.add(side.e[j],cross(W,o)));});out.push({m:pm.mass,E,off:Q.rot(this.q,V.add(side.wc,pm.offset))});}
      return out;
    }
    /* Lagrange ∂T/∂s_k at fixed (v, ω, ṡ): Σ m ẋ·(ω × e_k + Σ_j ∂e_j/∂s_k ṡ_j). */
    travelInertia(F){
      const eps=1e-6;
      for(let a=0;a<2;a++){
        const k0=2*a,sL=this.s[k0],sR=this.s[k0+1],base=this.massJacobians(a,sL,sR);
        const dE=[[this.massJacobians(a,sL+eps,sR),this.massJacobians(a,sL-eps,sR)],[this.massJacobians(a,sL,sR+eps),this.massJacobians(a,sL,sR-eps)]];
        base.forEach((pm,i)=>{
          const r=V.sub(pm.off,V.sub(this.p,this.p)),rw=pm.off;
          let xd=V.add(this.v,cross(this.w,rw));for(let j=0;j<2;j++)xd=V.add(xd,V.mul(pm.E[j],this.sd[k0+j]));
          for(let k=0;k<2;k++){
            let d=cross(this.w,pm.E[k]);
            for(let j=0;j<2;j++){const de=V.mul(V.sub(dE[k][0][i].E[j],dE[k][1][i].E[j]),1/(2*eps));d=V.add(d,V.mul(de,this.sd[k0+j]));}
            F[6+k0+k]+=pm.m*V.dot(xd,d);
          }
        });
      }
    }
    solve(g,out){const Mi=this.Minv;for(let i=0;i<N;i++){let s=0;for(let j=0;j<N;j++)s+=Mi[i*N+j]*g[j];out[i]=s;}return out;}
    applyU(du,s){for(let i=0;i<3;i++){this.v[i]+=du[i]*s;this.w[i]+=du[3+i]*s;}for(let j=0;j<4;j++)this.sd[j]+=du[6+j]*s;}
    /* Linear momentum of the whole vehicle / total mass = centre-of-mass velocity. */
    cmVelocity(){this.kinematics();this.buildMass();const u=[...this.v,...this.w,...this.sd],M=this.M,out=[0,0,0];for(let i=0;i<3;i++)for(let j=0;j<N;j++)out[i]+=M[i*N+j]*u[j];return out.map(x=>x/this.cfg.mass);}
    step(rawInput){
      if(this.invalid||this.boundary)return;
      // Preflight before any tick/input/environment mutation. The broad radius
      // covers the complete vehicle plus a whole step at the solver speed limit.
      if(this.surface.kind==='city'){
        const r=5+400/this.cfg.hz+Math.min(5,V.len(this.w)*2.5/this.cfg.hz);
        this.loading=!this.surface.isReady(this.p[0]-r,this.p[0]+r,this.p[2]-r,this.p[2]+r);
        if(this.loading)return false;
      }
      const input=cleanInput(rawInput);this.input=input;
      const c=this.cfg,dt=1/c.hz,oldV=this.v.slice(),oldP=this.p.slice();this.previous=this.pose();
      this.throttle=approach(this.throttle,input.throttle,dt*c.throttleRate);
      this.brake=approach(this.brake,input.brake,dt*c.brakeRate);
      this.powertrain.engine.charge=c.physicsProfile==='enhanced'?this.environment.chargeFactor(c.powertrain.engine.aspiration):1;
      for(const w of this.wheels){w.heatSlide=0;w.heatRoll=0;w.heatBrake=0;}
      this.steer=approach(this.steer,input.steer*c.steerMax*Math.PI/180,c.steerRate*Math.PI/180*dt);
      // Public steer input and HUD: negative LEFT, positive RIGHT.
      // Quaternion/geometric yaw about +Y has the opposite sign for a +Z-forward car.
      const wheelYaw=-this.steer;
      for(const wheel of this.wheels){
        if(wheel.sz<0||Math.abs(wheelYaw)<1e-7)wheel.steer=wheel.sz>0?wheelYaw:0;
        else {const radius=c.wheelbase/Math.tan(Math.abs(wheelYaw));const inner=wheel.sx===Math.sign(wheelYaw);wheel.steer=Math.sign(wheelYaw)*Math.atan(c.wheelbase/Math.max(.4,radius+(inner?-1:1)*c.track/2));}
      }
      // Steering changes wheel frames even when the suspension mass matrix is cached.
      this.kinematics();
      for(let i=0;i<c.substeps;i++)this.substep(dt/c.substeps,input);
      this.thermal.step(dt,this.wheels.map(w=>({slideW:w.heatSlide/dt,rollW:w.heatRoll/dt,brakeW:w.heatBrake/dt,speed:V.len(V.sub(this.v,this.environment.wind)),area:w.patch.areaSmooth,contact:w.contact})),this.environment);
      this.tick++;this.time=this.tick/c.hz;
      this.accel=V.mul(V.sub(this.v,oldV),1/dt);this.travel+=V.len(V.sub(this.p,oldP));
      this.kinematics();
      if(![...this.p,...this.v,...this.q,...this.w,...this.s,...this.sd,...this.wheels.map(w=>w.omega),this.powertrain.engine.omega,...this.thermal.stateVector(),this.diffHeatJ,this.diffProjectionJ].every(Number.isFinite)||V.len(this.v)>400||V.len(this.w)>150)this.invalid=true;
      if(this.surface.kind==='city'){
        const b=this.surface.bounds;if(this.p[0]<b[0][0]+2||this.p[0]>b[1][0]-2||this.p[2]<b[0][2]+2||this.p[2]>b[1][2]-2||this.p[1]<b[0][1]-8)this.boundary=true;
      }else if(Math.abs(this.p[0])>this.surface.half-2||Math.abs(this.p[2])>this.surface.half-2||this.p[1]<-8)this.boundary=true;
    }
    aeroForces(F){
      const c=this.cfg,rho=this.environment.airDensity,wind=this.environment.wind,fwd=Q.rot(this.q,[0,0,1]),up=Q.rot(this.q,[0,1,0]),yc=c.aeroHeight-c.cgHeight;
      const flow=rb=>{const r=Q.rot(this.q,rb),air=V.sub(this.pointVelocity(r),wind);return {r,air,speed:V.len(air)};};
      const add=(r,f)=>{F[0]+=f[0];F[1]+=f[1];F[2]+=f[2];F[3]+=r[1]*f[2]-r[2]*f[1];F[4]+=r[2]*f[0]-r[0]*f[2];F[5]+=r[0]*f[1]-r[1]*f[0];};
      // Forces sample the velocity of their OWN application point. Otherwise an
      // offset drag force can do positive work on a rotating vehicle in still air.
      const d=flow([0,yc,0]);this.aero.airspeed=d.speed;this.aero.drag=V.mul(d.air,-.5*rho*c.dragArea*d.speed);this.aero.lift=[0,0];add(d.r,this.aero.drag);
      for(const [k,cl,z]of [[0,c.liftFront,c.wheelbase/2],[1,c.liftRear,-c.wheelbase/2]]){
        if(!cl)continue;const a=flow([0,yc,z]);if(a.speed<1e-8)continue;
        // Lift is perpendicular to local air-relative flow; in still air it does
        // no generalized work, rather than spuriously adding vertical energy.
        const normal=V.sub(up,V.mul(a.air,V.dot(up,a.air)/(a.speed*a.speed))),n=V.len(normal);if(n<1e-8)continue;
        const u=V.dot(a.air,fwd),L=.5*rho*u*u*cl;this.aero.lift[k]=L;add(a.r,V.mul(normal,L/n));
      }
    }
    substep(h,input){
      const c=this.cfg,F=this.F;F.fill(0);
      if(!this.massValid){this.kinematics();this.buildMass();}this.massValid=false;
      for(const w of this.wheels)this.contactWheel(w);
      // ---- Explicit generalised forces, all from the pre-step state. ----
      this.environment.step(h);this.aeroForces(F);for(const w of this.wheels)this.surfaceMix(w);
      this.travelInertia(F);
      const ph=this.powertrain.health.suspension;
      for(let a=0;a<2;a++){
        const k0=2*a,hl=[0,1].map(i=>{const w=this.wheels[k0+i],e=w.eff;return {spring:ph[w.id]*e.spring,damper:ph[w.id]*e.damper,bumpStop:e.bumpStop};});
        const {Q:Qs,info}=this.axles[a].forces(this.s[k0],this.s[k0+1],0,0,hl,{bumpRate:c.bumpRate});
        F[6+k0]+=Qs[0];F[7+k0]+=Qs[1];this.wheels[k0].susp=info[0];this.wheels[k0+1].susp=info[1];
      }
      const g=this.tmp;
      for(const w of this.wheels){
        w.fx=0;w.fy=0;w.normalLoad=0;w.usage=0;w.force=[0,0,0];w.rimForce=0;w.slidePower=0;w.r=V.sub(w.point,this.p);
        if(!w.contact){w.brush=[0,0];w.slip=0;w.slipAngle=0;continue;}
        const st=w.patch,n=st.n,rate=-V.dot(this.pointVelocityOn(w,st.Pc),n),pinch=st.deltaMax-this.tyre.bottom;
        w.rimForce=pinch>0?Math.max(0,DMG.RIM_STIFFNESS*pinch+DMG.RIM_DAMPING*rate):0;
        w.normalLoad=Math.max(0,w.Kz*st.deltaEff+c.tyre.carcassDamping*rate)+w.rimForce;
        this.genVec(w,st.Pc,n,g);for(let i=0;i<N;i++)F[i]+=g[i]*w.normalLoad;
      }
      this.applyU(this.solve(F,new Float64Array(N)),h);this.v[1]-=c.gravity*h; // Mᐨ¹·(gravity) ≡ −g on v, exactly
      // ---- Powertrain, brakes and tyre friction: projected Gauss–Seidel. ----
      const pt=this.powertrain;
      pt.begin(h,input,this);
      const driven=new Set(pt.driven);
      for(const wheel of this.wheels){
        wheel.serviceTorque=pt.serviceBrakes.capacity(wheel,c,this.brake,pt.health)*this.thermal.brake(wheel.index);
        wheel.parkingTorque=pt.parkingBrake.capacity(wheel,c,input.handbrake,pt.health);
        const brakes=wheel.serviceTorque+wheel.parkingTorque;
        if(wheel.contact){const f=V.norm(V.sub(wheel.plane,V.mul(wheel.patch.n,V.dot(wheel.plane,wheel.patch.n))));this.surfaceGrip(wheel,Math.abs(V.dot(this.pointVelocityOn(wheel,wheel.patch.Pc),f)));}
        wheel.rrTorque=wheel.normalLoad*c.rollingResistance*wheel.road.rollingResistanceMultiplier*wheel.Re*(c.physicsProfile==='enhanced'?Math.pow(c.tyre.pressure/Math.max(40,wheel.pressure),.4):1);
        wheel.brakeLimit=(brakes+wheel.rrTorque)*h;wheel.brakeWorkJ=0;
        wheel.brakeImpulse=0;wheel.lambda=[0,0];wheel.driveVec=null;
        // Body-mounted differential: halfshaft torque also works on the carrier's side-view pitch.
        if(driven.has(wheel)&&this.axles[wheel.axle].type.kind!=='solid'){
          const gd=new Float64Array(N);let any=false;for(let j=0;j<4;j++){gd[6+j]=V.dot(wheel.axis,wheel.WW[j]);any=any||gd[6+j]!==0;}
          if(any)wheel.driveVec=this.solve(gd,new Float64Array(N));
        }
        if(!wheel.contact||wheel.normalLoad<=0)continue;
        const n=wheel.patch.n,P=wheel.patch.Pc;
        wheel.forward=V.norm(V.sub(wheel.plane,V.mul(n,V.dot(wheel.plane,n))));wheel.side=V.cross(wheel.forward,n); // driver-right tangent
        wheel.gf=this.genVec(wheel,P,wheel.forward,new Float64Array(N));wheel.gs=this.genVec(wheel,P,wheel.side,new Float64Array(N));
        wheel.Mf=this.solve(wheel.gf,new Float64Array(N));wheel.Ms=this.solve(wheel.gs,new Float64Array(N));
        const vx=V.dot(this.pointVelocityOn(wheel,P),wheel.forward);
        // Camber thrust (brush model): equivalent lateral slip velocity |vx|·(a / 3R)·sin γ.
        wheel.camberSlip=Math.abs(vx)*wheel.patch.halfLength/(3*this.tyre.R0)*V.dot(wheel.axis,n);
        // Slip stiffness follows the footprint. Tread brush term C_t = 2·c_p·w·a² (brush theory) in series with a
        // load-independent carcass term, equal at the reference load: C/C_ref = 2ρ/(1+ρ), ρ = A·a / (A·a)_ref
        // (w·a² = A·a/2 for a rectangle; A = N/(κp) membrane area, a from the pressure second moment).
        // Carcass stiffness K is unchanged, so the relaxation length σ = C/K carries the load dependence.
        const rho=wheel.patch.areaSmooth*wheel.patch.halfLength/this.patch0,g=Math.max(.05,2*rho/(1+rho));wheel.stiffnessRatio=g;
        const transportX=Math.abs(vx)/(c.relaxationX*g),transportY=Math.abs(vx)/(c.relaxationY*g);
        wheel.D=[1+h*transportX,1+h*transportY];
        wheel.B=[c.brushKx*h/wheel.D[0]+c.brushDamping,c.brushKy*h/wheel.D[1]+c.brushDamping];
        wheel.C=[c.brushKx*wheel.brush[0]/wheel.D[0],c.brushKy*wheel.brush[1]/wheel.D[1]];
        wheel.gamma=wheel.B.map(v=>1/(h*v));
        const ref=c.mass*c.gravity/4;
        wheel.mu=c.mu*wheel.road.frictionMultiplier*this.thermal.grip(wheel.index)*Math.pow(ref/Math.max(ref*.15,wheel.normalLoad),c.loadSensitivity);
        // Sliding friction falls below peak as sliding speed rises (rubber on asphalt; exponential decay as in the
        // Penn State skid model, floor r = μ_locked/μ_peak). Applied only to sliding speed beyond the brush model's
        // full-slide onset |vx|·3μN/C, so peak grip below the limit is unchanged; spinning, locked and drifting tyres lose μ.
        {const vsx=wheel.Re*wheel.omega-vx,vsy=wheel.camberSlip-V.dot(this.pointVelocityOn(wheel,P),wheel.side),Vs=Math.hypot(vsx,vsy);
         if(Vs>1e-6){const N0=wheel.mu*wheel.normalLoad,fx=Math.max(.05,Math.abs(vx)*3*N0/(c.brushKx*c.relaxationX*g)),fy=Math.max(.05,Math.abs(vx)*3*N0/(c.brushKy*c.relaxationY*g));
          const sn=Math.hypot(vsx/fx,vsy/fy),excess=sn>1?Vs*(1-1/sn):0;wheel.slideSpeed=excess;wheel.mu*=wheel.road.slideFloor+(1-wheel.road.slideFloor)*Math.exp(-excess/(wheel.slideRef||SLIDE_SPEED));} else wheel.slideSpeed=0;}
        wheel.limit=wheel.mu*wheel.normalLoad*h;
        wheel.A=[dot10(wheel.gf,wheel.Mf)+wheel.Re*wheel.Re/c.wheelInertia+wheel.gamma[0],dot10(wheel.gs,wheel.Ms)+wheel.gamma[1],dot10(wheel.gf,wheel.Ms)];
      }
      const diffPairs=[];
      if(c.diff.type!=='open')for(let a=0;a<2;a++){const L=this.wheels[2*a],R=this.wheels[2*a+1];if(driven.has(L)&&driven.has(R)){const Tin=Math.abs(pt.clutch.lastTorque)/(c.drive==='AWD'?2:1),lock=pt.clutch.lastTorque*pt.ratio>=0?c.diff.power:c.diff.coast;diffPairs.push({L,R,lambda:0,workJ:0,limit:c.diff.type==='locked'?Infinity:(c.diff.preload+lock*Tin)*h});}}
      this.prepareSuspensionDamping(h,ph);for(let it=0;it<c.iterations;it++){
        for(const d of diffPairs)DIFF.project(d,c.wheelInertia);
        const before=pt.clutch.lambda;
        pt.solve(h,this);
        const dj=pt.clutch.lambda-before;
        if(dj&&pt.ratio){const share=pt.ratio*dj/pt.driven.length;for(const w of pt.driven)if(w.driveVec)this.applyU(w.driveVec,share);}
        for(let j=0;j<4;j++){
          const wheel=this.wheels[it%2?3-j:j];
          if(wheel.brakeLimit>0){const nb=clamp(wheel.brakeImpulse-wheel.omega*c.wheelInertia,-wheel.brakeLimit,wheel.brakeLimit);const j=nb-wheel.brakeImpulse;wheel.brakeWorkJ-=j*(wheel.omega+.5*j/c.wheelInertia);wheel.omega+=j/c.wheelInertia;wheel.brakeImpulse=nb;}
          if(!wheel.contact||wheel.normalLoad<=0)continue;
          const vp=this.pointVelocityOn(wheel,wheel.patch.Pc);
          const slipX=wheel.Re*wheel.omega-V.dot(vp,wheel.forward),slipY=wheel.camberSlip-V.dot(vp,wheel.side);
          const [a,b,x]=wheel.A,det=a*b-x*x;
          const rhsX=slipX+wheel.C[0]/wheel.B[0]-wheel.gamma[0]*wheel.lambda[0];
          const rhsY=slipY+wheel.C[1]/wheel.B[1]-wheel.gamma[1]*wheel.lambda[1];
          let nx=wheel.lambda[0]+(b*rhsX-x*rhsY)/det,ny=wheel.lambda[1]+(a*rhsY-x*rhsX)/det;
          const magnitude=Math.hypot(nx,ny);if(magnitude>wheel.limit){nx*=wheel.limit/magnitude;ny*=wheel.limit/magnitude;}
          const dx=nx-wheel.lambda[0],dy=ny-wheel.lambda[1];wheel.lambda=[nx,ny];
          for(let i=0;i<3;i++){this.v[i]+=wheel.Mf[i]*dx+wheel.Ms[i]*dy;this.w[i]+=wheel.Mf[3+i]*dx+wheel.Ms[3+i]*dy;}
          for(let k=0;k<4;k++)this.sd[k]+=wheel.Mf[6+k]*dx+wheel.Ms[6+k]*dy;
          wheel.omega-=dx*wheel.Re/c.wheelInertia;
        }
        this.iterateSuspensionDamping();
      }
      this.finishSuspensionDamping();pt.finish(h);this.diffTorque=diffPairs.map(d=>d.lambda/h);const diffLoss=diffPairs.reduce((n,d)=>n+Math.max(0,d.workJ),0);if(c.diff.type==='clutch')this.diffHeatJ+=diffLoss;else if(c.diff.type==='locked')this.diffProjectionJ+=diffLoss;
      for(const wheel of this.wheels){
        if(wheel.contact&&wheel.normalLoad>0){
          const vp=this.pointVelocityOn(wheel,wheel.patch.Pc),vx=V.dot(vp,wheel.forward),vy=V.dot(vp,wheel.side),sx=wheel.Re*wheel.omega-vx;
          wheel.brush=[(wheel.brush[0]+h*sx)/wheel.D[0],(wheel.brush[1]+h*(wheel.camberSlip-vy))/wheel.D[1]];
          // Plastic return mapping: don't accumulate unlimited stored rubber shear while sliding.
          const elasticMag=Math.hypot(c.brushKx*wheel.brush[0],c.brushKy*wheel.brush[1]);
          const elasticLimit=wheel.mu*wheel.normalLoad;
          if(elasticMag>elasticLimit)wheel.brush=wheel.brush.map(v=>v*elasticLimit/elasticMag);
          wheel.fx=wheel.lambda[0]/h;wheel.fy=wheel.lambda[1]/h;
          wheel.slip=sx/Math.max(1,Math.abs(vx),Math.abs(wheel.Re*wheel.omega));
          wheel.slipAngle=Math.atan2(vy,Math.max(.5,Math.abs(vx)));
          wheel.usage=Math.hypot(...wheel.lambda)/Math.max(1e-9,wheel.limit);
          // Frictional power at the contact (tyre force · sliding velocity): rubber deposition.
          wheel.slidePower=Math.abs(wheel.fx*sx)+Math.abs(wheel.fy*(vy-wheel.camberSlip));
          wheel.force=V.add(V.mul(wheel.patch.n,wheel.normalLoad),V.add(V.mul(wheel.forward,wheel.fx),V.mul(wheel.side,wheel.fy)));
        } else wheel.force=[0,0,0];
        {const energy=Math.max(0,wheel.brakeWorkJ),brakes=wheel.serviceTorque+wheel.parkingTorque,share=brakes+wheel.rrTorque>0?brakes/(brakes+wheel.rrTorque):0;wheel.heatBrake+=energy*share;wheel.heatRoll+=energy*(1-share);wheel.heatSlide+=wheel.slidePower*h;}
        wheel.angle+=wheel.omega*h;
      }
      this.solveHullGround(h);
      // Generalised momentum at this configuration, plus −v×P for angular momentum about the moving body point.
      const u=[...this.v,...this.w,...this.sd],Pm=new Float64Array(N);
      for(let i=0;i<N;i++){let s=0;for(let j=0;j<N;j++)s+=this.M[i*N+j]*u[j];Pm[i]=s;}
      const vxP=cross(this.v,[Pm[0],Pm[1],Pm[2]]);Pm[3]-=h*vxP[0];Pm[4]-=h*vxP[1];Pm[5]-=h*vxP[2];
      this.p=V.add(this.p,V.mul(this.v,h));this.q=Q.integrate(this.q,this.w,h);
      for(let k=0;k<4;k++)this.s[k]+=this.sd[k]*h;
      this.updateDamage(h);
      this.kinematics();this.buildMass();this.massValid=true;
      const un=this.solve(Pm,new Float64Array(N));for(let i=0;i<3;i++){this.v[i]=un[i];this.w[i]=un[3+i];}for(let k=0;k<4;k++)this.sd[k]=un[6+k];
    }
    updateDamage(h){
      const qc=Q.conj(this.q);
      for(const w of this.wheels){
        const Fb=Q.rot(qc,w.force),own=w.eB[w.index],Wd=w.WB[w.index];
        const Pb=Q.rot(qc,V.sub(w.point,this.p)),e=V.norm(V.add(own,cross(Wd,V.sub(Pb,w.wcBody))));
        const link=V.sub(Fb,V.mul(e,V.dot(Fb,e))),st=w.patch; // load the travel DOF cannot take goes through the links
        this.damage.update(w.index,{link:[-w.sx*link[0],-link[2]],rimForce:w.rimForce,pinch:st.contact?Math.max(0,st.deltaMax-this.tyre.bottom):0,
          contactAngle:this.tyre.theta[st.iMax]+w.angle,damperForce:w.susp.damper,damperSpeed:w.susp.damperSpeed,bumpStop:w.susp.bumpStop,volume:this.tyre.volume},h,w.id);
      }
    }
    solveHullCity(h){
      const c=this.cfg,axes=[[1,0,0],[0,1,0],[0,0,1]].map(a=>Q.rot(this.q,a)),offset=Q.rot(this.q,[0,.87-c.cgHeight,0]),center=V.add(this.p,offset);
      const angularMargin=Math.min(5,V.len(this.w)*2.5*h),raw=this.surface.bodyContacts({center,axes,halfExtents:[.88,.5,2.06],margin:.003+angularMargin,displacement:V.mul(this.v,h),maxContacts:16});
      const load=(r,d)=>{const g=new Float64Array(N),rc=cross(r,d);g[0]=d[0];g[1]=d[1];g[2]=d[2];g[3]=rc[0];g[4]=rc[1];g[5]=rc[2];const M=this.solve(g,new Float64Array(N));return {M,e:dot10(g,M)};};
      const contacts=raw.map(co=>{
        const r=V.sub(co.point,this.p),n=co.normal,t1=V.norm(V.cross(Math.abs(n[1])<.9?[0,1,0]:[1,0,0],n)),t2=V.cross(n,t1);
        return {...co,r,n,t1,t2,normal:load(r,n),u:load(r,t1),v:load(r,t2),lambda:0,fric:[0,0]};
      });
      this.bodyContacts=contacts.length;
      for(let k=0;k<6;k++)for(const co of contacts){
        let vel=this.pointVelocity(co.r),vn=V.dot(vel,co.n),target=Math.min(3,.18*Math.max(0,co.depth-.002)/h);
        // Swept contacts stop inward velocity before crossing a thin wall. No
        // restitution or positional teleport; normal impulses act through M^-1.
        const next=Math.max(0,co.lambda+(target-vn)/co.normal.e),dj=next-co.lambda;
        if(k===0&&vn<0&&dj>0)this.damage.hull(.5*dj*-vn);
        this.applyU(co.normal.M,dj);co.lambda=next;vel=this.pointVelocity(co.r);
        let a=co.fric[0]-V.dot(vel,co.t1)/co.u.e,b=co.fric[1]-V.dot(vel,co.t2)/co.v.e;
        const size=Math.hypot(a,b),limit=.6*next;if(size>limit){a*=limit/size;b*=limit/size;}
        this.applyU(co.u.M,a-co.fric[0]);this.applyU(co.v.M,b-co.fric[1]);co.fric=[a,b];
      }
    }
    solveHullGround(h){
      if(this.surface.kind==='city')return this.solveHullCity(h);
      // Limited collision scope: the eight corners of an illustrative hull against the road surface.
      // Positional bias is numerical penetration recovery, not a vehicle handling force.
      const c=this.cfg,contacts=[],g=new Float64Array(N),m=new Float64Array(N);
      const load=(r,d)=>{g.fill(0);g[0]=d[0];g[1]=d[1];g[2]=d[2];const rc=cross(r,d);g[3]=rc[0];g[4]=rc[1];g[5]=rc[2];return this.solve(g,m);};
      for(const x of [-.88,.88])for(const y of [.37,1.37])for(const z of [-2.06,2.06]){
        const r=Q.rot(this.q,[x,y-c.cgHeight,z]),p=V.add(this.p,r),ground=this.surface.heightAt(p[0],p[2]);
        if(p[1]<ground+.003&&this.surface.onPad(p[0],p[2])){const en=dot10(g,load(r,[0,1,0])),ex=dot10(g,load(r,[1,0,0])),ez=dot10(g,load(r,[0,0,1]));contacts.push({r,depth:Math.max(0,ground-p[1]),lambda:0,fric:[0,0],en,ex,ez});}
      }
      this.bodyContacts=contacts.length;
      for(let k=0;k<5;k++)for(const co of contacts){
        let vel=this.pointVelocity(co.r);const target=Math.min(3,.18*Math.max(0,co.depth-.002)/h);
        const nj=Math.max(0,co.lambda+(target-vel[1])/co.en);
        if(k===0&&vel[1]<0)this.damage.hull(.5*(nj-co.lambda)*-vel[1]);
        this.applyU(load(co.r,[0,1,0]),nj-co.lambda);co.lambda=nj;
        vel=this.pointVelocity(co.r);
        let fx=co.fric[0]-vel[0]/co.ex,fz=co.fric[1]-vel[2]/co.ez;
        const n=Math.hypot(fx,fz),lim=.6*nj;if(n>lim){fx*=lim/n;fz*=lim/n;}
        this.applyU(load(co.r,[1,0,0]),fx-co.fric[0]);this.applyU(load(co.r,[0,0,1]),fz-co.fric[1]);co.fric=[fx,fz];
      }
    }
    telemetry(){
      const f=Q.rot(this.q,[0,0,1]),right=Q.rot(this.q,[-1,0,0]),up=Q.rot(this.q,[0,1,0]);
      return {tick:this.tick,time:this.time,speed:V.len(this.v)*3.6,forwardSpeed:V.dot(this.v,f)*3.6,
        pitch:Math.asin(clamp(f[1],-1,1))*180/Math.PI,roll:Math.atan2(right[1],up[1])*180/Math.PI,
        yawRate:-this.w[1]*180/Math.PI,longG:V.dot(this.accel,f)/this.cfg.gravity,latG:V.dot(this.accel,right)/this.cfg.gravity,
        height:this.p[1],distance:this.travel,environment:{...this.environment.snapshot(),airDensity:this.environment.airDensity},powertrain:this.powertrain.telemetry(),load:this.wheels.reduce((s,w)=>s+w.normalLoad,0),
        contacts:this.wheels.filter(w=>w.contact&&w.normalLoad>1).length,
        wheels:this.wheels.map(w=>{const st=w.patch,d=this.damage.corners[w.index];return {id:w.id,material:w.material||'asphalt',road:w.road?{...w.road}:this.environment.conditions(),load:w.normalLoad,compression:w.compression,length:w.length,
          rpm:w.omega*60/(2*Math.PI),slip:w.slip,slipAngle:w.slipAngle*180/Math.PI,fx:w.fx,fy:w.fy,usage:w.usage,contact:w.contact,
          camber:w.camber*180/Math.PI,inclination:-w.sx*Math.asin(clamp(w.axis?w.axis[1]:0,-1,1))*180/Math.PI,toe:w.toe*180/Math.PI,
          deflection:st.deltaEff,patchLength:st.length,patchWidth:st.width,patchArea:st.area,contactPressure:st.area>0?w.normalLoad/st.area:0,
          tyrePressure:d.pressure,livePressure:w.pressure,treadC:this.thermal.Ts[w.index]-273.15,carcassC:this.thermal.Tc[w.index]-273.15,gasC:this.thermal.Tg[w.index]-273.15,brakeC:this.thermal.Tb[w.index]-273.15,thermalGrip:this.thermal.grip(w.index),brakeFade:this.thermal.brake(w.index),water:w.water||0,hydro:w.hydro||0,surfaceMu:w.surfaceMu??1,Kz:w.Kz,rimForce:w.rimForce,slidePower:w.slidePower,spring:w.susp.spring,damper:w.susp.damper,
          damage:{set:Math.hypot(...d.set),camber:d.set[0]/DMG.KINGPIN*180/Math.PI,toe:-d.set[1]/DMG.STEER_ARM*180/Math.PI,failed:!!d.failed,damper:d.damper,bumpStop:d.bumpStop,hole:d.hole,dent:d.dent,peakLink:d.peakLink,peakRim:d.peakRim}};}),
        chassisDamageJ:this.damage.chassisJ,airDensity:this.environment.airDensity,charge:this.powertrain.engine.charge,differential:{heatJ:this.diffHeatJ,projectionJ:this.diffProjectionJ,torques:this.diffTorque.slice()}};
    }
    axleMetrics(){return this.axles.map((a,i)=>a.metrics(this.static[i],this.cfg.cgHeight,this.cfg.wheelbase,this.cfg.frontBrakeBias,this.cfg.drive!=='FWD'));}
    repair(){this.damage.reset(this.cfg.tyre.pressure);this.massValid=false;this.refreshVisualContacts();}
    stateVector(){return [...this.p,...this.v,...this.q,...this.w,this.steer,this.throttle,this.brake,...this.powertrain.stateVector(),...this.wheels.flatMap(w=>[w.omega,w.angle,...w.brush]),...this.s,...this.sd,...this.damage.stateVector(),...this.environment.stateVector(),...this.thermal.stateVector(),this.diffHeatJ,this.diffProjectionJ];}
    exportState(){return {version:VERSION,config:PT.clone(this.cfg),tick:this.tick,p:this.p.slice(),v:this.v.slice(),q:this.q.slice(),w:this.w.slice(),steer:this.steer,throttle:this.throttle,brake:this.brake,travel:this.travel,powertrain:this.powertrain.snapshot(),
      wheels:this.wheels.map(w=>({omega:w.omega,angle:w.angle,brush:w.brush.slice()})),s:this.s.slice(),sd:this.sd.slice(),damage:this.damage.snapshot(),environment:this.environment.snapshot(),thermal:this.thermal.snapshot(),differential:{heatJ:this.diffHeatJ,projectionJ:this.diffProjectionJ}};}
    importState(s){
      if(!s||![VERSION,'0.4.2','0.4.1','0.4.0','0.3.0'].includes(s.version))throw new Error('Snapshot version differs from this physics build.');
      if(s.version===VERSION&&!Number.isFinite(s.powertrain?.clutch?.lastTorque))throw Error('Missing replay differential torque state');
      const config=validateConfig(s.config);if(s.version===VERSION){requireResolved(s.config,config,'config');requireResolved(s.environment?.cfg,ENV.validate(s.environment?.cfg),'environment.cfg');}
      this.cfg=config;this.build();this.reset();
      for(const key of ['p','v','q','w','s','sd']){const n=key==='p'||key==='v'||key==='w'?3:4;if(!Array.isArray(s[key])||s[key].length!==n||!s[key].every(Number.isFinite))throw new Error('Invalid snapshot '+key);this[key]=s[key].slice();}
      if(Math.abs(Math.hypot(...this.q)-1)>1e-6)throw new Error('Snapshot orientation must be a unit quaternion.');
      if(!Number.isInteger(s.tick)||s.tick<0)throw new Error('Snapshot tick must be a non-negative integer.');
      this.tick=s.tick;this.time=this.tick/this.cfg.hz;
      for(const key of ['steer','throttle','brake','travel']){if(!Number.isFinite(s[key]))throw new Error('Invalid snapshot '+key);if((['throttle','brake'].includes(key)&&(s[key]<0||s[key]>1))||(key==='travel'&&s[key]<0)||(key==='steer'&&Math.abs(s[key])>Math.PI/2))throw new Error('Out-of-range snapshot '+key);this[key]=s[key];}
      if(!Array.isArray(s.wheels)||s.wheels.length!==4)throw new Error('Invalid snapshot wheels.');
      s.wheels.forEach((w,i)=>{if(![w.omega,w.angle,...(w.brush||[])].every(Number.isFinite)||w.brush?.length!==2)throw new Error('Invalid wheel state');this.wheels[i].omega=w.omega;this.wheels[i].angle=w.angle;this.wheels[i].brush=w.brush.slice();});
      this.powertrain.restore(s.powertrain);this.damage.restore(s.damage);if(s.version!=='0.3.0')this.environment.restore(s.environment);if(s.version===VERSION){this.thermal.restore(s.thermal);if(!s.differential||!['heatJ','projectionJ'].every(k=>Number.isFinite(s.differential[k])&&s.differential[k]>=0))throw Error('Invalid differential state');this.diffHeatJ=s.differential.heatJ;this.diffProjectionJ=s.differential.projectionJ;}else if(s.thermal)this.thermal.restore(s.thermal);this.massValid=false;
      this.refreshVisualContacts();this.previous=this.pose();return this;
    }
  }
  const scenarioInput=(name,t)=>{
    const i=emptyInput();
    if(name==='launch'){if(t>1&&t<7)i.throttle=.8;if(t>=7)i.brake=.65;}
    if(name==='turn'){if(t>1&&t<5)i.throttle=.55;if(t>=5&&t<13){i.throttle=.20;i.steer=.17;}if(t>=11)i.brake=.22;}
    if(name==='slalom'){i.throttle=t>1&&t<6?.5:t<19?.18:0;i.steer=t>5&&t<19?.16*Math.sin((t-5)*1.3):0;if(t>=19)i.brake=.55;}
    if(name==='demo'){if(t>1&&t<5)i.throttle=.65;if(t>=5&&t<14){i.throttle=.22;i.steer=.19;}if(t>=14&&t<18){i.throttle=.25;i.steer=-.15;}if(t>=18)i.brake=.6;}
    if(name==='surface'){if(t>1&&t<3.5)i.throttle=.6;else if(t>=3.5&&t<39)i.throttle=.21;if(t>=39)i.brake=.55;}
    if(name==='curb'){if(t>1&&t<4.8)i.throttle=.9;if(t>=8.5)i.brake=.7;}
    // Test driver supplies a real clutch pedal profile (manual mode also supported).
    if(name!=='drop'){
      if(t<1.35)i.clutch=1;
      else if(t<2.1)i.clutch=1-(t-1.35)/.75;
      if(i.brake>.3)i.clutch=1;
    }
    return i;
  };
  {
    // Implicit suspension damping. At each frozen configuration, dampers satisfy
    // lambda = -h C G u. Solve the four rows simultaneously within the existing PGS
    // iterations, removing the current accumulated lambda before each block solve.
    // No physical rates, timestep, iteration count or state bounds are changed.
    const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
    function factorSPD(A){
      const n=A.length,L=Array.from({length:n},()=>Array(n).fill(0));
      for(let i=0;i<n;i++)for(let j=0;j<=i;j++){
        let x=A[i][j];for(let k=0;k<j;k++)x-=L[i][k]*L[j][k];
        if(i===j){if(!(x>0))throw new Error('Damper matrix is not positive definite');L[i][j]=Math.sqrt(x);}
        else L[i][j]=x/L[j][j];
      }
      return L;
    }
    function solveFactored(L,b){
      const n=b.length,y=b.slice(),x=b.slice();
      for(let i=0;i<n;i++){for(let j=0;j<i;j++)y[i]-=L[i][j]*y[j];y[i]/=L[i][i];}
      for(let i=n-1;i>=0;i--){x[i]=y[i];for(let j=i+1;j<n;j++)x[i]-=L[j][i]*x[j];x[i]/=L[i][i];}
      return x;
    }
    function rowsFor(v,ph){
      const rows=[];
      for(let a=0;a<2;a++)for(let i=0;i<2;i++){
        const axle=v.axles[a],k=2*a+i,w=v.wheels[k],g=Array(10).fill(0);
        if(axle.type.kind==='solid'){
          g[6+2*a]=(1+(i?-1:1)*axle.rho)/2;
          g[7+2*a]=(1-(i?-1:1)*axle.rho)/2;
        }else g[6+k]=axle.type.kind==='independent'?w.susp.MR:1;
        const coefficient=axle.cd*ph[w.id]*w.eff.damper;
        rows.push({g,coefficient,w});
      }
      return rows;
    }
    function install(Vehicle){Object.assign(Vehicle.prototype,{
      prepareSuspensionDamping(h,ph){
        const rows=rowsFor(this,ph);
        for(const row of rows){row.response=Array.from(this.solve(row.g,new Float64Array(10)));row.scale=Math.sqrt(h*row.coefficient);}
        const K=rows.map(r=>rows.map(s=>dot(r.g,s.response))),A=rows.map((r,i)=>rows.map((s,j)=>(i===j?1:0)+r.scale*K[i][j]*s.scale));
        this.damperBlock={h,rows,K,L:factorSPD(A),lambda:[0,0,0,0]};
      },
      iterateSuspensionDamping(){
        const {rows,K,L,lambda}=this.damperBlock,u=[...this.v,...this.w,...this.sd];
        const free=rows.map((r,i)=>dot(r.g,u)-dot(K[i],lambda)),x=solveFactored(L,rows.map((r,i)=>r.scale*free[i]));
        for(let i=0;i<4;i++){const j=-rows[i].scale*x[i];this.applyU(rows[i].response,j-lambda[i]);lambda[i]=j;}
      },
      finishSuspensionDamping(){
        const {rows}=this.damperBlock,u=[...this.v,...this.w,...this.sd];
        for(const r of rows){const speed=dot(r.g,u);r.w.susp.damper=r.coefficient*speed;r.w.susp.damperSpeed=speed;}
      },
    });}
    install(Vehicle);

  }
  root.TarmacPhysics={Vehicle,DEFAULTS,PROFILES,BOUNDS,VERSION,validateConfig,emptyInput,cleanInput,scenarioInput,SCENARIO_START};
})(globalThis);

