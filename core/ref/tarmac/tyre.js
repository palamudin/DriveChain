/* === tyre.js === */
/* Tyre carcass and contact geometry. No DOM or solver state.
 * Geometry from the ISO metric size code (205/55 R16): section width W, aspect ratio, rim diameter.
 * Radial stiffness from inflation pressure and size (Rhyne 2005, Tire Sci. Tech. 33(3), as applied in
 * Jazar, Vehicle Dynamics, 2017):  Kz [kgf/mm] = 0.00028 · p[kPa] · sqrt(Wcp[mm] · D[mm]) + 3.45
 *   Wcp = (1.03 − 0.004 · AR) · W  (contact width estimate), D = unloaded outer diameter.
 *   The pressure term is the membrane contribution; 3.45 is the structural (0 kPa) stiffness.
 * Contact: a fan of radial rays in the wheel plane (dense under the patch, sparse ahead/behind for
 * obstacles) repeated across the tread in lanes. Each ray measures penetration of the undeformed
 * tread circle into the road. The penetrated volume is mapped back to the flat-road deflection that
 * would produce the same volume, so the radial load is exactly Kz·δ on a flat road and the tyre
 * envelops features shorter than its footprint.
 * Rubber contact: the belt is not a rigid circle; most of δ is sidewall compression (the belt ring
 * moves toward the rim) and only the rest flattens the tread. The footprint is the region of the
 * geometric overlap deeper than that ring displacement, sized so contact area = N / (κ · p)
 * (membrane equilibrium; κ = mean contact pressure / inflation pressure). Pressure within it is
 * overlap minus ring displacement: parabolic along the patch, the brush-model assumption.
 */
(function(root){
'use strict';
const DEG=Math.PI/180,clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const FLANGE=.0175,PINCH_RUBBER=.012; // ETRTO J-flange height; compressed tread + casing, two layers
function parseSize(text){
 const m=/^\s*(\d{3})\s*\/\s*(\d{2})\s*Z?R\s*(\d{2}(?:\.\d)?)\s*$/i.exec(String(text));
 if(!m)throw new Error('Tyre size must look like 205/55 R16.');
 const W=+m[1],AR=+m[2],rim=+m[3];
 if(W<135||W>355)throw new Error('Section width must be 135 … 355 mm.');
 if(AR<25||AR>85)throw new Error('Aspect ratio must be 25 … 85.');
 if(rim<12||rim>22)throw new Error('Rim diameter must be 12 … 22 in.');
 const width=W/1000,H=width*AR/100,rimR=rim*.0254/2,R0=rimR+H;
 return {code:`${W}/${AR} R${rim}`,width,aspect:AR/100,rimRadius:rimR,H,R0};
}
const contactWidth=(widthM,aspect)=>(1.03-.004*aspect*100)*widthM;
function rhyneKz(pressureKPa,widthM,aspect,diameterM){return (.00028*Math.max(0,pressureKPa)*Math.sqrt(contactWidth(widthM,aspect)*1000*diameterM*1000)+3.45)*9.81*1000;}
const TYRE_DEFAULTS={size:'205/55 R16',pressure:220,contactPressureRatio:.8,carcassDamping:300,carcassRoll:.6,lanes:7};
function validateTyre(raw){
 const out={...TYRE_DEFAULTS};if(raw==null)return out;
 if(typeof raw!=='object'||Array.isArray(raw))throw new Error('tyre must be an object.');
 if('size' in raw){parseSize(raw.size);out.size=parseSize(raw.size).code;}
 const B={pressure:[60,400],contactPressureRatio:[.4,1.4],carcassDamping:[0,2000],carcassRoll:[0,1.5]};
 for(const [k,[lo,hi]]of Object.entries(B))if(k in raw){const x=Number(raw[k]);if(!Number.isFinite(x)||x<lo||x>hi)throw new Error(`tyre.${k} must be ${lo} … ${hi}.`);out[k]=x;}
 return out;
}
class Tyre {
 constructor(spec){
  this.spec=spec;Object.assign(this,parseSize(spec.size));
  this.D=2*this.R0;this.tread=contactWidth(this.width,this.aspect);this.pressure=spec.pressure;
  this.Kz=rhyneKz(spec.pressure,this.width,this.aspect,this.D);this.cz=spec.carcassDamping;
  this.bottom=this.H-FLANGE-PINCH_RUBBER; // radial deflection at which the rim flange pinches the casing
  this.volume=2*Math.PI*(this.rimRadius+this.H/2)*(this.width*this.H*.75); // air volume, torus approximation
  // Ray fan: dense ±33° at 1.5° under the patch; sparse 37–65° at 4° to sense kerbs early.
  const th=[];for(let a=-65;a<-35;a+=4)th.push(a);for(let a=-33;a<=33.0001;a+=1.5)th.push(+a.toFixed(3));for(let a=37;a<=65.0001;a+=4)th.push(a);
  this.theta=Float64Array.from(th.map(a=>a*DEG));this.nr=th.length;this.cos=this.theta.map(Math.cos);this.sin=this.theta.map(Math.sin);
  this.dense=[th.indexOf(-33),th.indexOf(33)];
  this.w=new Float64Array(this.nr);for(let i=0;i<this.nr;i++){const a=this.theta[Math.max(0,i-1)],b=this.theta[Math.min(this.nr-1,i+1)];this.w[i]=this.R0*(b-a)/(i===0||i===this.nr-1?1:2);}
  this.nl=spec.lanes;this.laneW=this.tread/this.nl;this.lane=Float64Array.from({length:this.nl},(_,j)=>(j+.5)*this.laneW-this.tread/2);
  // Flat-road volume V(δ) per unit lane width is piecewise linear in δ: invert exactly.
  const bp=[...this.theta.keys()].map(i=>({d:this.R0*(1-this.cos[i]),k:this.w[i]/this.cos[i]})).sort((a,b)=>a.d-b.d);
  this.bpD=[];this.bpS=[];this.bpT=[];this.bpV=[];let S=0,T=0;
  for(const b of bp){S+=b.k;T+=b.k*b.d;this.bpD.push(b.d);this.bpS.push(S);this.bpT.push(T);this.bpV.push(0);}
  for(let k=0;k<bp.length;k++)this.bpV[k]=k===0?0:this.bpS[k-1]*this.bpD[k]-this.bpT[k-1];
  this.grid=null;
 }
 kz(pressureKPa){return rhyneKz(pressureKPa,this.width,this.aspect,this.D);}
 flatVolume(delta){let V=0;for(let i=0;i<this.nr;i++){const p=this.R0-(this.R0-delta)/this.cos[i];if(p>0)V+=p*this.w[i];}return V;}
 invert(V){
  if(V<=0)return 0;const n=this.bpD.length;let lo=0,hi=n-1;
  if(V>=this.bpV[n-1])return (V+this.bpT[n-1])/this.bpS[n-1];
  while(hi-lo>1){const mid=(lo+hi)>>1;if(this.bpV[mid]<=V)lo=mid;else hi=mid;}
  return (V+this.bpT[lo])/this.bpS[lo];
 }
 /* Geometric patch half-length on a flat road for a deflection δ (for checks and display). */
 chord(delta){return Math.sqrt(Math.max(0,2*this.R0*delta-delta*delta));}
 makeState(){return {grid:new Float32Array(this.nl*this.nr),q:new Float32Array(this.nl*this.nr),ring:0,cells:[],t:new Float32Array(this.nl*this.nr),origins:new Float64Array(this.nl*3),dirs:new Float64Array(this.nr*3),
  n:[0,1,0],Pc:[0,0,0],contact:false,cellCount:0,deltaEff:0,deltaMax:0,iMax:0,length:0,width:0,area:0,lanesIn:0,Vmean:0,shift:[0,0,0]};}
 /* Cast the fan. g: {c, a (axle, left), d (down in wheel plane), f (forward in wheel plane),
  * s (ground side, driver-right), shiftY (tread lateral offset along s, m), shiftX (along f),
  * roll (carcass roll angle, rad), dent {depth, angle, width}, spin (wheel angle),
  * Kz (N/m) and pressure (kPa gauge) for the footprint area}. */
 contact(g,surface,st){
  const {c,a,d,f,s}=g,R0=this.R0,nl=this.nl,nr=this.nr,hit={n:[0,1,0]};
  st.contact=false;st.cellCount=0;st.deltaEff=0;st.deltaMax=0;st.Vmean=0;st.length=0;st.width=0;st.area=0;st.lanesIn=0;st.ring=0;st.halfLength=0;st.areaSmooth=0;st.grid.fill(0);st.q.fill(0);
  const box=R0+this.tread,info=surface.near(c[0]-box,c[0]+box,c[2]-box,c[2]+box),plane=!info.any;
  if(c[1]-info.top>R0+.03||!surface.onPad(c[0],c[2])){surface.release();st.t.fill(0);return st;}
  const as=a[0]*s[0]+a[1]*s[1]+a[2]*s[2];
  for(let i=0;i<nr;i++){st.dirs[i*3]=this.cos[i]*d[0]+this.sin[i]*f[0];st.dirs[i*3+1]=this.cos[i]*d[1]+this.sin[i]*f[1];st.dirs[i*3+2]=this.cos[i]*d[2]+this.sin[i]*f[2];}
  let Vsum=0,dmax=0,imax=0,jmax=0,nc=0;const cells=st.cells;
  for(let j=0;j<nl;j++){
   const y=this.lane[j],e=-(y*as)*g.roll;
   const ox=c[0]+a[0]*y+s[0]*g.shiftY+f[0]*g.shiftX+d[0]*e,oy=c[1]+a[1]*y+s[1]*g.shiftY+f[1]*g.shiftX+d[1]*e,oz=c[2]+a[2]*y+s[2]*g.shiftY+f[2]*g.shiftX+d[2]*e;
   st.origins[j*3]=ox;st.origins[j*3+1]=oy;st.origins[j*3+2]=oz;
   for(let i=0;i<nr;i++){
    const Dx=st.dirs[i*3],Dy=st.dirs[i*3+1],Dz=st.dirs[i*3+2];
    let R=R0;if(g.dent&&g.dent.depth>0){let q=this.theta[i]+g.spin-g.dent.angle;q-=2*Math.PI*Math.round(q/(2*Math.PI));R-=g.dent.depth*Math.max(0,1-Math.abs(q)/g.dent.width);}
    let t;if(plane){t=Dy<-1e-9?oy/(-Dy):-1;if(t>R0*1.6)t=-1;}else t=surface.raycast(ox,oy,oz,Dx,Dy,Dz,R0*1.6,hit,false);
    st.t[j*nr+i]=t;
    if(t<0)continue;const pen=R-t;if(pen<=0)continue;
    st.grid[j*nr+i]=pen;Vsum+=pen*this.w[i];
    if(pen>dmax){dmax=pen;imax=i;jmax=j;}
    let cell=cells[nc];if(!cell)cell=cells[nc]={k:0,pen:0,area:0,x:0,y:0,z:0,nx:0,ny:1,nz:0};
    cell.k=j*nr+i;cell.pen=pen;cell.area=this.w[i]*Math.abs(Dy)*this.laneW;cell.x=ox+Dx*t;cell.y=oy+Dy*t;cell.z=oz+Dz*t;
    cell.hit=!plane&&hit.role?{materialId:hit.materialId,role:hit.role}:null;
    if(plane){cell.nx=0;cell.ny=1;cell.nz=0;}else{cell.nx=hit.n[0];cell.ny=hit.n[1];cell.nz=hit.n[2];}
    nc++;
   }
  }
  surface.release();st.cellCount=nc;
  if(nc===0)return st;
  st.contact=true;st.Vmean=Vsum/nl;st.deltaEff=this.invert(st.Vmean);st.deltaMax=dmax;st.iMax=imax;st.jMax=jmax;
  // Rubber footprint: deepest overlap first until area = N / (κ p). The threshold is the belt-ring displacement.
  const Ns=g.Kz*st.deltaEff,target=g.pressure>1?Ns/(this.spec.contactPressureRatio*g.pressure*1000):Infinity;
  // Smoothed area (each cell ramps in over 0.4 mm) keeps the threshold continuous in load; bisection is deterministic.
  const eps=.0004,areaAt=tau=>{let A=0;for(let k=0;k<nc;k++){const cl=cells[k],x=(cl.pen-tau)/eps+.5;A+=x>=1?cl.area:x>0?cl.area*x:0;}return A;};
  let ring=0;if(areaAt(0)>target){let lo=0,hi=dmax+eps;for(let it=0;it<34;it++){const mid=.5*(lo+hi);if(areaAt(mid)>target)lo=mid;else hi=mid;}ring=.5*(lo+hi);}
  st.ring=ring;
  let sw=0,px=0,py=0,pz=0,nx=0,ny=0,nz=0,A=0,fmin=Infinity,fmax=-Infinity,lmin=Infinity,lmax=-Infinity,m1=0,m2=0;
  const as2=a[0]*c[0]+a[1]*c[1]+a[2]*c[2],fs=f[0]*c[0]+f[1]*c[1]+f[2]*c[2];
  for(let k=0;k<nc;k++){const cl=cells[k],qv=cl.pen-ring;if(qv<=0)continue;st.q[cl.k]=qv;const wt=qv*cl.area;sw+=wt;A+=cl.area;
   px+=wt*cl.x;py+=wt*cl.y;pz+=wt*cl.z;nx+=wt*cl.nx;ny+=wt*cl.ny;nz+=wt*cl.nz;
   const lf=f[0]*cl.x+f[1]*cl.y+f[2]*cl.z-fs,la=a[0]*cl.x+a[1]*cl.y+a[2]*cl.z-as2;
   // Each ray represents a finite longitudinal strip, not a point mass. Its
   // horizontal projected area is cl.area = stripLength * laneW. Treat the
   // sampled pressure as constant inside that strip: Var(x) = stripLength²/12.
   // This is the same finite-volume quadrature used for force/area above;
   // retaining its intrinsic moment also resolves a one-ray pressure patch.
   const stripLength=cl.area/this.laneW;
   m1+=wt*lf;m2+=wt*(lf*lf+stripLength*stripLength/12);if(lf<fmin)fmin=lf;if(lf>fmax)fmax=lf;if(la<lmin)lmin=la;if(la>lmax)lmax=la;}
  if(sw<=0){st.contact=false;return st;}
  const nn=Math.hypot(nx,ny,nz);st.n[0]=nx/nn;st.n[1]=ny/nn;st.n[2]=nz/nn;st.Pc[0]=px/sw;st.Pc[1]=py/sw;st.Pc[2]=pz/sw;
  st.area=A;st.length=fmax-fmin+R0*1.5*DEG;
  // Continuous measures for the tyre forces: half-length from the pressure second moment (parabolic: Var = a²/5),
  // area from membrane equilibrium. Both vary smoothly with state; the cell extents above are for display.
  st.halfLength=Math.sqrt(Math.max(0,5*(m2/sw-(m1/sw)**2)));st.areaSmooth=Math.min(target,areaAt(0));st.width=Math.min(this.tread,lmax-lmin+this.laneW);
  let lanes=0;for(let j=0;j<nl;j++){for(let i=0;i<nr;i++)if(st.q[j*nr+i]>0){lanes++;break;}}st.lanesIn=lanes;
  return st;
 }
}
root.TarmacTyre={Tyre,parseSize,rhyneKz,contactWidth,validateTyre,TYRE_DEFAULTS,FLANGE,PINCH_RUBBER};
})(globalThis);

