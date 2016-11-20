using UnityEngine;
using System.Collections;
using UnityEditor;

[CustomEditor(typeof(Helicopter))]
public class HeliEditor : Editor {

	public override void OnInspectorGUI() {
		DrawDefaultInspector();
		Helicopter heli = (Helicopter)target;
		if (GUILayout.Button("Load Default")) {
			heli.LoadDefault();
		}
	}

}
