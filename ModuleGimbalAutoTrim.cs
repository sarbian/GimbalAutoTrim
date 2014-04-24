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

            if (this.gimbalAutoTrim)
                AlignGimbal();
            else
                UnalignGimbal();
        }

        [KSPField(isPersistant = true)]
        public bool gimbalAutoTrim = false;

        [KSPField(isPersistant = false)]
        public float trimLimit = 45;

        public override string GetInfo()
        {
            return base.GetInfo();
        }

        [KSPEvent(guiName = "Auto-Trim Gimbal", guiActive = true, guiActiveEditor = true)]
        public void AlignGimbal()
        {
            this.gimbalAutoTrim = true;
            this.Events["AlignGimbal"].active = false;
            this.Events["UnalignGimbal"].active = true;
        }

        [KSPEvent(guiName = "Disable Auto-Trim Gimbal", guiActive = true, guiActiveEditor = true)]
        public void UnalignGimbal()
        {
            this.gimbalAutoTrim = false;
            this.Events["AlignGimbal"].active = true;
            this.Events["UnalignGimbal"].active = false;

            for (int i = 0; i < this.initalRots.Count(); i++)
            {
                this.initRots[i] = initalRots[i];
            }
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
                        // ModuleGimbal.initRots is protected :(
                        List<Quaternion> initRots = (List<Quaternion>)(typeof(ModuleGimbal).GetField("initRots", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance).GetValue(gimbal));
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
                    Vector3 optimalDir = ti.cotAll - ti.com;
                    Vector3 currentDir = ti.dotOther + ti.dotAligned;

                    Vector3 correction = Vector3.Exclude(optimalDir, currentDir);

                    float x = Mathf.Min(correction.magnitude, ti.thrustAligned);

                    float y = Mathf.Sqrt(ti.thrustAligned * ti.thrustAligned - x * x);

                    Vector3 trimedDotAligned = -correction.normalized * x + optimalDir.normalized * y;

                    float trimAngle = Vector3.Angle(ti.dotAligned, trimedDotAligned);

                    Quaternion trimRotation = Quaternion.FromToRotation(ti.dotAligned, trimedDotAligned);

                    //print("optimalDir " + PrettyPrint(optimalDir) + " currentDir " + PrettyPrint(currentDir) + " correction " + PrettyPrint(correction) + " angle " + Vector3.Angle(currentDir, optimalDir).ToString("F2") + " cAngle " + trimAngle.ToString("F3"));

                    for (int i = 0; i < this.initRots.Count(); i++)
                    {
                        float trimRatio = Mathf.Min(trimLimit / trimAngle, 1f);
                        
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

            base.OnFixedUpdate();
        }

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
