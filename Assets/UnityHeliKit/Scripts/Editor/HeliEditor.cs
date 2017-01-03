using UnityEngine;
using System.Collections;
using UnityEditor;

[CustomEditor(typeof(SingleMainRotorHelicopter))]
public class HeliEditor : Editor {

	public override void OnInspectorGUI() {
		DrawDefaultInspector();
		SingleMainRotorHelicopter heli = (SingleMainRotorHelicopter)target;
		if (GUILayout.Button("Load Default")) {
			heli.LoadDefault();
		}
	}

}
