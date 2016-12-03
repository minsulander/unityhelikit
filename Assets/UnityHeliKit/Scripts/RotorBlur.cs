using UnityEngine;
using System.Collections;

public class RotorBlur : MonoBehaviour {

	public enum Source {
		MainRotor,
		TailRotor
	}
	Source source = Source.MainRotor;

    public MeshRenderer[] blades;
    public MeshRenderer[] blurs;
	public float upperRotorBlurLimit = 15f;
	public float lowerRotorBlurLimit = 8f;
    private float blur;


	private Helicopter helicopter;

	void Start () {
		helicopter = GetComponentInParent<Helicopter>();
	}
	
	void Update () {
		var rotspeed = source == Source.MainRotor ? helicopter.model.MainRotor.RotSpeed : helicopter.model.TailRotor.RotSpeed;
		if (rotspeed < lowerRotorBlurLimit) blur = 0;
		else if (rotspeed > upperRotorBlurLimit) blur = 1;
		else blur = ((float)rotspeed - lowerRotorBlurLimit) / (upperRotorBlurLimit - lowerRotorBlurLimit);

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
