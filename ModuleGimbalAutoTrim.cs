/*
 * Author: Sébastien GAGGINI AKA Sarbian, France
 * License: Attribution 4.0 International (CC BY 4.0): http://creativecommons.org/licenses/by/4.0/
 *  
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;


namespace GimbalAutoTrim
{
    [KSPModule("Auto-Trim")]
    public class ModuleGimbalAutoTrim : ModuleGimbal
    {

        // Backup of the backup of the default engine rotation
        public List<Quaternion> initalRots;

        public struct ThrustInfo
        {
            public Vector3 com;          // Center of Mass
            public Vector3 cotAll;       // Center of Thrust for all Engines
            public Vector3 dotOther;     // Direction of Thrust for engines without AutoTrim Active
            public Vector3 dotAligned;   // Direction of Thrust for engines with AutoTrim Active
            public float thrustOther;    // Total Thrust or engines without AutoTrim Active
            public float thrustAligned;  // Total Thrust or engines with AutoTrim Active
        }

        private static float lastFixedTime = 0;
        private static Dictionary<Guid, ThrustInfo> vesselsThrustInfo = new Dictionary<Guid, ThrustInfo>();

        public override void OnStart(PartModule.StartState state)
        {
            base.OnStart(state);

            initalRots = new List<Quaternion>();
            foreach (Quaternion q in initRots)
                initalRots.Add(q);
        }

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Auto-Trim"),
        UI_Toggle(scene = UI_Scene.All)]
        public bool gimbalAutoTrim = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "Auto-Trim Limit"),
        UI_FloatRange(minValue = 0f, maxValue = 90f, scene = UI_Scene.Editor, stepIncrement = 5f)]
        public float trimLimit = 45f;

        [KSPField(isPersistant = false, guiActive = false, guiName = "Angle")]
        public string trimStatus = "";

        [KSPField(isPersistant = false, guiActive = true, guiName = "correction")]
        public string correction = "";

        [KSPField(isPersistant = false, guiActive = true, guiName = "trimAngle")]
        public string trimAngle = "";

        public override string GetInfo()
        {
            return base.GetInfo();
        }

        public ThrustInfo updateThrust()
        {
            ThrustInfo ti = new ThrustInfo();

            ti.cotAll = Vector3.zero;
            ti.dotOther = Vector3.zero;
            ti.dotAligned = Vector3.zero;
            ti.thrustOther = 0;
            ti.thrustAligned = 0;

            foreach (Part p in vessel.Parts)
            {
                Vector3 coT = Vector3.zero;
                Vector3 doT = Vector3.zero;
                float thrust = 0;

                Vector3 thurstForward = Vector3.zero;

                ModuleEngines engine = p.Modules.OfType<ModuleEngines>().FirstOrDefault();
                ModuleEnginesFX enginefx = p.Modules.OfType<ModuleEnginesFX>().Where(e => e.isEnabled).FirstOrDefault();

                if (enginefx != null || engine != null)
                {
                    List<Transform> thrustTransforms;

                    ModuleGimbal gimbal = p.Modules.OfType<ModuleGimbal>().FirstOrDefault();

                    if (engine != null)
                        thrustTransforms = engine.thrustTransforms;
                    else // ModuleEnginesFX
                        thrustTransforms = enginefx.thrustTransforms;

                    List<Vector3> forwards;
                    if (gimbal != null)
                    {
                        List<Quaternion> initRots = gimbal.initRots;
                        forwards = new List<Vector3>(initRots.Count);
                        for (int i = 0; i < initRots.Count; i++)
                        {
                            Transform tt = thrustTransforms[i];
                            Quaternion ori = tt.localRotation;
                            tt.localRotation = (gimbal is ModuleGimbalAutoTrim) ? (gimbal as ModuleGimbalAutoTrim).initalRots[i] : initRots[i];
                            forwards.Add(tt.forward);
                            tt.localRotation = ori;
                        }
                    }
                    else
                    {
                        forwards = new List<Vector3>(thrustTransforms.Count);
                        foreach (Transform thrustTransform in thrustTransforms)
                            forwards.Add(thrustTransform.forward);
                    }

                    for (int i = 0; i < thrustTransforms.Count; i++)
                        foreach (Transform thrustTransform in thrustTransforms)
                        {
                            coT = coT + (thrustTransforms[i].position - p.transform.position);
                            doT = doT + forwards[i];
                        }
                    coT = (coT / (float)thrustTransforms.Count) + p.transform.position;
                    doT = doT / (float)thrustTransforms.Count;

                    thrust = (engine != null) ? engine.finalThrust : enginefx.finalThrust;

                    if (gimbal != null && (gimbal is ModuleGimbalAutoTrim) && (gimbal as ModuleGimbalAutoTrim).gimbalAutoTrim)
                    {
                        ti.dotAligned += doT * thrust;
                        ti.thrustAligned += thrust;
                    }
                    else
                    {
                        ti.dotOther += doT * thrust;
                        ti.thrustOther += thrust;
                    }
                    ti.cotAll += coT * thrust;
                }

            }
            ti.cotAll = ti.cotAll / (ti.thrustOther + ti.thrustAligned);
            ti.com = this.vessel.findWorldCenterOfMass();

            return ti;
        }
        
        public override void OnFixedUpdate()
        {
            if (HighLogic.LoadedSceneIsEditor || this.vessel == null)
                return;

            Fields["trimStatus"].guiActive = this.gimbalAutoTrim;

            if (this.gimbalAutoTrim)
            {
                ThrustInfo ti;
                // If we moved to a new phyisic frame then clear the cache
                if (lastFixedTime != Time.fixedTime) 
                {
                    vesselsThrustInfo = new Dictionary<Guid, ThrustInfo>();
                    lastFixedTime = Time.fixedTime;
                }
                if (!vesselsThrustInfo.ContainsKey(vessel.id))
                {
                    ti = updateThrust();
                    vesselsThrustInfo.Add(vessel.id, ti);
                }
                else
                    ti = vesselsThrustInfo[vessel.id];
                                
                if (ti.thrustAligned > 0f)
                {
                    Vector3 optimalDot = ti.cotAll - ti.com;
                    Vector3 currentDot = ti.dotOther + ti.dotAligned;
                    
                    // CoT in front of CoM
                    if (Vector3.Dot(optimalDot, currentDot) < 0f)
                        optimalDot = -optimalDot;
                    
                    // We work in a 2D plane whose axis are correction and optimalDot 
                    // correction is the perpendicular to optimalDot in the plane defined by optimalDot/currentDot 
                    Vector3 correction = Vector3.Exclude(optimalDot, currentDot);
                    this.correction = PrettyPrint(correction,"F2");

                    float thrustAlignedX = Vector3.Exclude(optimalDot, ti.dotAligned).magnitude;

                    float x = Mathf.Clamp(thrustAlignedX - correction.magnitude, -ti.thrustAligned, ti.thrustAligned);

                    float y = Mathf.Sqrt(ti.thrustAligned * ti.thrustAligned - x * x);

                    Vector3 trimedDotAligned = correction.normalized * x + optimalDot.normalized * y;
                    
                    float trimAngle = Vector3.Angle(ti.dotAligned, trimedDotAligned);
                    this.trimAngle = trimAngle.ToString("F2");

                    Quaternion trimRotation = Quaternion.FromToRotation(ti.dotAligned, trimedDotAligned);

                    //print("optimalDir " + PrettyPrint(optimalDir) + " currentDir " + PrettyPrint(currentDir) + " correction " + PrettyPrint(correction) + " angle " + Vector3.Angle(currentDir, optimalDir).ToString("F2") + " cAngle " + trimAngle.ToString("F3"));

                    float trimRatio = Mathf.Min(trimLimit / trimAngle, 1f);
                    
                    this.trimStatus = (trimAngle * trimRatio).ToString("F1") + " deg";

                    for (int i = 0; i < this.initRots.Count(); i++)
                    {   
                        gimbalTransforms[i].localRotation = initalRots[i];
                        gimbalTransforms[i].forward = Quaternion.Lerp( Quaternion.identity, trimRotation, trimRatio) * gimbalTransforms[i].forward;
                        this.initRots[i] = gimbalTransforms[i].localRotation;
                    }
                }
                else
                {
                    for (int i = 0; i < this.initalRots.Count(); i++)
                    {
                        this.initRots[i] = initalRots[i];
                    }
                }
            }
            else
            {
                for (int i = 0; i < this.initalRots.Count(); i++)
                {
                    this.initRots[i] = initalRots[i];
                }
            }

            base.OnFixedUpdate();
        }


        //public override void OnUpdate()
        //{
            
        //    base.OnUpdate();
        //}

        public static string PrettyPrint(Vector3d vector, string format = "F3")
        {
            return "[" + (vector.x >= 0 ? " " : "") + vector.x.ToString(format) + ", " + (vector.y >= 0 ? " " : "") + vector.y.ToString(format) + ", " + (vector.z >= 0 ? " " : "") + vector.z.ToString(format) + "]";
        }

        public static new void print(object message)
        {
            MonoBehaviour.print("[ModuleGimbalAligned] " + message);
        }


    }
}
