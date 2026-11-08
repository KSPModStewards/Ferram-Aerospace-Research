
namespace FerramAerospaceResearch.FARAeroComponents
{
    public sealed class DummyAirstreamShield : IAirstreamShield
    {
        /// <inheritdoc />
        public bool ClosedAndLocked()
        {
            return true;
        }

        public Part part;

        /// <inheritdoc />
        public Vessel GetVessel()
        {
            return part.vessel;
        }

        /// <inheritdoc />
        public Part GetPart()
        {
            return part;
        }
    }
}
