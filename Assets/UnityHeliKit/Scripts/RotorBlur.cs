using UnityEngine;
using System.Collections;

public class RotorBlur : MonoBehaviour {

    public MeshRenderer[] blades;
    public MeshRenderer[] blurs;
    public float blur;

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	    foreach (MeshRenderer mesh in blades) {
            if (blur > 0.99f) {
                mesh.enabled = false;
            } else {
                mesh.enabled = true;
                if (mesh.material != null) {
                    Color color = mesh.material.color;
                    color.a = 1f - blur;
                    mesh.material.SetColor("_Color", color);
                }
                foreach (Material material in mesh.materials) {
                    if (!material.HasProperty("_Color")) continue;
                    Color color = material.color;
                    color.a = 1f - blur;
                    material.SetColor("_Color", color);
                }
            }
        }
        foreach (MeshRenderer mesh in blurs) {
            if (blur < 0.01f) {
                mesh.enabled = false;
            } else {
                mesh.enabled = true;
                foreach (Material material in mesh.materials) {
                    if (!material.HasProperty("_Color")) continue;
                    Color color = material.color;
                    color.a = blur;
                    material.SetColor("_Color", color);
                }
            }
        }
	}
}
