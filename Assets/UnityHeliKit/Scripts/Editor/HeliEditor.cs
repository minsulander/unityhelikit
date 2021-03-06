﻿using UnityEngine;
using System.Collections;
using UnityEditor;
using Newtonsoft.Json;
using System.IO;

[CustomEditor(typeof(Helicopter), true)]
public class HeliEditor : Editor {

	public override void OnInspectorGUI() {
		DrawDefaultInspector();
		Helicopter heli = (Helicopter)target;
		if (GUILayout.Button("Load Default")) {
			heli.model.LoadDefault();
            heli.FindComponents();
            heli.ParametrizeUnityFromModel();
        }
        /*
        if (GUILayout.Button("Open Model")) {
            var path = EditorUtility.OpenFilePanel("Open helicopter model", "", "helisharp");
            heli.model = JsonConvert.DeserializeObject<HeliSharp.SingleMainRotorHelicopter>(File.ReadAllText(path));
            heli.FindComponents();
            heli.ParametrizeUnityFromModel();
        }
        */
        if (GUILayout.Button("Save Model")) {
            heli.ParametrizeModelsFromUnity();
            var json = JsonConvert.SerializeObject(heli.model, Newtonsoft.Json.Formatting.Indented);
            var path = EditorUtility.SaveFilePanel("Save helicopter model", "", heli.name, "helisharp");
            File.WriteAllText(path, json);
        }
    }

}
