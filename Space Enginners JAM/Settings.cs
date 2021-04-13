using VRageMath;

namespace IngameScript {
    partial class Program {
        public class Settings {
            public const string TAG = "JAM";
            public const bool AutoStartThrusters = true;
            public const float CASDistanceExtension = 1.0f;
            public const float MinRayDist = 0.0f;
            public const float MinThrustEffectivness = 0.05f; // When the effectivness of the thruster drops below gets deactivated
            public const float ApproachHeight = 100.0f;
            public const float APPROACH_SPEED = 5.0f;
            public const float DEPARTURE_SPEED = 10.0f;
            public const float DOCKING_SPEED = 1.0f;
            public const float MAX_SPEED = 95.0f;
            public const float ARRIVAL_DIST = 1000.0f;
            public const float MIN_CHECK_DIST = 1e-1f;
            public const float SCAN_ANGLE = 15.0f;
            public const string COMM_CHANNEL = "JAM_CH_1";
            public const float EPSILON = 1e-3f;
            public const float MIN_LOOKAT_DIST = 10.0f;
            public const int STORAGE_VER = 112;
            public const int LIST_EXTENTS = 3;
            public static Vector3D v_epsilon = new Vector3D(EPSILON);
            public static Vector3D[] ManagedThrusterDirections = { -Vector3D.UnitZ, Vector3D.UnitZ, Vector3D.UnitY, -Vector3D.UnitY, -Vector3D.UnitX, Vector3D.UnitX };
        }
    }
}