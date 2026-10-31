namespace FerramAerospaceResearch
{
    public class Version
    {
        // using byte here because 0-255 should be enough for any version number
        public const byte Major = 0;
        public const byte Minor = 16;
        public const byte Build = 2;
        public const byte Revision = 0;
        public const string Name = "Maskell";

        /// <summary>
        /// String of the numerical version
        /// </summary>
        public static readonly string ShortString = $"v{Major}.{Minor}.{Build}.{Revision}";

        /// <summary>
        /// String of the numerical version with name
        /// </summary>
        public static readonly string LongString = $"{ShortString} '{Name}'";

        /// <summary>
        /// String of the build commit in CI
        /// </summary>
        public const string GitSha = null;
    }
}
