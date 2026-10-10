#!/usr/bin/env python3
"""Read-only CSV comparison; inferred labels from ROS vs system clock.
Usage: python3 analyze.py [input_directory] [output_directory]
"""
import csv,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
SRC=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/mnt/hgfs/vmware共享文件夹')
OUT=Path(sys.argv[2]) if len(sys.argv)>2 else Path(__file__).resolve().parent
OUT.mkdir(parents=True,exist_ok=True)
files=sorted(SRC.glob('trotting_debug_5121_*.csv'))+sorted(SRC.glob('trotting_debug_6695_*.csv'))
def read(p):
 with p.open() as f:
  r=csv.reader(f);h=next(r);a=np.asarray(list(r),dtype=float)
 return {k:a[:,i] for i,k in enumerate(h)}
def mat(d,k,n):return np.column_stack([d[f'{k}_{i}'] for i in range(n)])
def rms(x):return float(np.sqrt(np.nanmean(np.asarray(x)**2)))
def pct(x,p=95):return float(np.nanpercentile(x,p))
def summary(d,mask):
 q=mat(d,'q',12);qd=mat(d,'qd',12);qc=mat(d,'q_cmd',12);qdc=mat(d,'qd_cmd',12)
 t=d['ros_s']; valid=(np.diff(d['cycle'])==1)&(np.diff(t)>0)&(np.diff(t)<.0121)&mask[1:]&mask[:-1]
 fd=np.diff(q,axis=0)/np.diff(t)[:,None]
 qdv=.5*(qd[1:]+qd[:-1]); rpy=mat(d,'rpy',3); rpyr=mat(d,'rpy_ref',3)
 kp=mat(d,'kp_cmd',12);kd=mat(d,'kd_cmd',12);ff=mat(d,'tau_ff_cmd',12);fb=mat(d,'tau_feedback',12);mit=mat(d,'tau_mit_est',12)
 contact=mat(d,'contact',4).astype(bool);swing=np.repeat(~contact,3,axis=1)&mask[:,None];stance=np.repeat(contact,3,axis=1)&mask[:,None]
 err=(rpy-rpyr+np.pi)%(2*np.pi)-np.pi
 out={'n':int(mask.sum()),'p_z_mean_m':float(d['p_2'][mask].mean()),'p_z_std_m':float(d['p_2'][mask].std()),'p_z_error_mean_m':float((d['p_2']-d['p_ref_2'])[mask].mean()),'p_ref_z_mean_m':float(d['p_ref_2'][mask].mean()),'p_z_error_rmse_m':rms((d['p_2']-d['p_ref_2'])[mask]),'rpy_error_rmse_deg':np.rad2deg(np.sqrt(np.mean(err[mask]**2,axis=0))).tolist(),'rpy_std_deg':np.rad2deg(rpy[mask].std(axis=0)).tolist(),'rpy_mean_deg':np.rad2deg(rpy[mask].mean(axis=0)).tolist(),'qd_rms':rms(qd[mask]),'qd_p99_abs':pct(abs(qd[mask]),99),'qd_vs_fd_rmse':rms((qdv-fd)[valid]),'fd_rms':rms(fd[valid]),'torque_feedback_rms_Nm':rms(fb[mask]),'torque_mit_rms_Nm':rms(mit[mask]),'torque_same_time_rmse_Nm':rms((fb-mit)[mask]),'feedback_abs_p99_Nm':pct(abs(fb[mask]),99),'mit_abs_p99_Nm':pct(abs(mit[mask]),99),'mit_over_100_fraction':float((abs(mit[mask])>100).mean()),'calf_above_urdf_upper_fraction':float((q[mask,2::3]>-.888).mean()),'solver_result_counts':{str(v):int((d['solver_result'][mask]==v).sum()) for v in np.unique(d['solver_result'][mask])},'wall_dt_p50_p99_ms':[pct(d['wall_dt_ms'][mask],50),pct(d['wall_dt_ms'][mask],99)],'ros_dt_p50_p99_ms':[pct(np.diff(t)[valid]*1000,50),pct(np.diff(t)[valid]*1000,99)],'update_p99_max_ms':[pct(d['update_ms'][mask],99),float(d['update_ms'][mask].max())],'v_xyz_rmse_mps':[rms(d[f'v_{i}'][mask]-d[f'v_ref_{i}'][mask]) for i in range(3)],'acc_G_std_mps2':mat(d,'acc_G',3)[mask].std(axis=0).tolist(),'force_sum_z_mean_N':float(d['force_sum_2'][mask].mean()),'contact_mean_per_leg':contact[mask].mean(axis=0).tolist(),'tau_feedback_unique_per_joint':[int(len(np.unique(fb[mask,j]))) for j in range(12)]}
 for name,m in [('swing',swing),('stance',stance)]:
  out[name]={'q_error_rms_rad':rms((qc-q)[m]),'qd_error_rms_rad_s':rms((qdc-qd)[m]),'tau_ff_rms_Nm':rms(ff[m]),'tau_p_rms_Nm':rms((kp*(qc-q))[m]),'tau_d_rms_Nm':rms((kd*(qdc-qd))[m]),'tau_feedback_rms_Nm':rms(fb[m])}
 fe=(mat(d,'feet_G',12)-mat(d,'goal_G',12)).reshape(-1,4,3)
 out['swing_foot_error_xyz_rmse_m']=[rms(fe[:,:,i][(~contact)&mask[:,None]]) for i in range(3)]
 # Lag score is diagnostic only: closed-loop + different torque conventions prevent identification.
 out['feedback_vs_mit_lag_scores']=[]
 for lag in range(0,6):
  a=fb[lag:] if lag else fb;b=mit[:-lag] if lag else mit
  m=mask[lag:]&mask[:-lag] if lag else mask.copy()
  if lag:m&=(d['cycle'][lag:]-d['cycle'][:-lag])==lag
  out['feedback_vs_mit_lag_scores'].append({'lag_samples':lag,'rmse_Nm':rms((a-b)[m]),'corr':float(np.corrcoef(a[m].ravel(),b[m].ravel())[0,1])})
 return out
runs=[];totals=[]
for p in files:
 d=read(p);runs.append((p,d));m=np.ones(len(d['cycle']),bool)
 s=summary(d,m);s['file']=p.name;s['clock_label']='simulation_inferred' if d['ros_s'][0]<1e6 else 'hardware_inferred';s['cycle_start_end']=[int(d['cycle'][0]),int(d['cycle'][-1])];s['ros_duration_s']=float(d['ros_s'][-1]-d['ros_s'][0]);s['steady_duration_s']=float(d['steady_s'][-1]-d['steady_s'][0]);s['velocity_ref_abs_max']=float(abs(mat(d,'v_ref',3)).max());totals.append(s)
matched=[]
for p,d in runs:
 if d['cycle'][0]!=0:continue
 m=(d['cycle']>=500)&(d['cycle']<=2500)
 s=summary(d,m);s['file']=p.name;s['window']='cycles 500..2500 (2..10 s)';matched.append(s)
(OUT/'metrics.json').write_text(json.dumps({'per_file':totals,'matched_first_window':matched},indent=2,allow_nan=False))
rows=[]
for s in totals+matched:
 row={k:v for k,v in s.items() if isinstance(v,(str,int,float))};row['window']=s.get('window','whole_file');rows.append(row)
keys=list(dict.fromkeys(k for r in rows for k in r))
with (OUT/'metrics.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
colors=['#d55e00','#0072b2'];fig,axes=plt.subplots(4,2,figsize=(14,12),sharex=True)
for idx,pid in enumerate(['5121','6695']):
 subset=[(p,d) for p,d in runs if f'_{pid}_' in p.name];origin=subset[0][1]['ros_s'][0]
 for fi,(p,d) in enumerate(subset):
  t=d['ros_s']-origin;c=colors[idx];lab=('Gazebo (5121)' if idx==0 else 'Hardware (6695)') if fi==0 else None
  axes[0,0].plot(t,d['p_2'],c=c,label=lab);axes[0,1].plot(t,(d['p_2']-d['p_ref_2'])*100,c=c,label=lab)
  axes[1,0].plot(t,np.rad2deg(d['rpy_0']-d['rpy_ref_0']),c=c,label=lab);axes[1,1].plot(t,np.rad2deg(d['rpy_1']-d['rpy_ref_1']),c=c,label=lab)
  axes[2,0].plot(t,np.rad2deg((d['rpy_2']-d['rpy_ref_2']+np.pi)%(2*np.pi)-np.pi),c=c,label=lab)
  axes[2,1].plot(t,d['v_2'],c=c,alpha=.8,label=lab)
  axes[3,0].plot(t,d['qd_2'],c=c,alpha=.8,label=lab)
  axes[3,1].plot(t,d['tau_feedback_2'],c=c,alpha=.8,label=lab)
for ax,title in zip(axes.ravel(),['Estimated body height (m)','Estimated height - reference (cm)','Roll error (deg)','Pitch error (deg)','Yaw error (deg)','Estimated vertical velocity (m/s)','FR calf velocity feedback (rad/s)','FR calf effort feedback (Nm)']):
 ax.set_title(title);ax.grid(alpha=.25);ax.legend(fontsize=8)
for ax in axes[-1]:ax.set_xlabel('ROS elapsed time since first recorded cycle (s)')
fig.suptitle('Trotting logs: zero target velocity; gaps are unrecorded windows.\nBody/foot states are estimates; effort conventions differ between simulation and hardware.')
fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(3,2,figsize=(14,9),sharex='col')
for idx,(p,d) in enumerate([(p,d) for p,d in runs if d['cycle'][0]==0]):
 m=(d['cycle']>=1000)&(d['cycle']<=1250);t=d['ros_s']-d['ros_s'][0];q=mat(d,'q',12);fd=np.zeros_like(q);fd[1:]=np.diff(q,axis=0)/np.diff(d['ros_s'])[:,None]
 axes[0,idx].plot(t[m],d['qd_2'][m],label='Velocity feedback');axes[0,idx].plot(t[m],fd[m,2],label='Angle difference / dt',alpha=.75)
 axes[1,idx].plot(t[m],d['tau_feedback_2'][m],label='Effort feedback');axes[1,idx].plot(t[m],d['tau_mit_est_2'][m],label='Interface MIT torque estimate',alpha=.75)
 for k in ['tau_ff_cmd_2','tau_p_est_2','tau_d_est_2']:axes[2,idx].plot(t[m],d[k][m],label=k)
 axes[0,idx].set_title(('Gazebo 5121' if idx==0 else 'Hardware 6695')+' / FR calf')
for i in range(3):
 for ax in axes[i]:ax.grid(alpha=.25);ax.legend(fontsize=8);ax.set_ylabel(['rad/s','Nm','Nm'][i])
for ax in axes[-1]:ax.set_xlabel('Elapsed ROS time (s)')
fig.tight_layout();fig.savefig(OUT/'joint_detail.png',dpi=170);plt.close(fig)
for s in matched:
 print(s['file']);print(json.dumps(s,indent=2))
print('Per-file height/reference/std/yaw/qd consistency:')
for s in totals:print(s['file'],s['p_z_mean_m'],s['p_ref_z_mean_m'],s['p_z_std_m'],s['rpy_std_deg'][2],s['qd_vs_fd_rmse'])
