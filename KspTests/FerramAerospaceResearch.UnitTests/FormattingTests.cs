using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using StringLeakTest;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// The garbage-free number formatting FAR's flight readouts use (StringBuilderExtNumeric), against .NET's own, and
    /// the config strings' placeholder formatter (StringFormatter).
    /// </summary>
    public class FormattingTests
    {
        private static string Of(Func<StringBuilder, StringBuilder> concat) => concat(new StringBuilder()).ToString();

        [Theory]
        [InlineData(0)]
        [InlineData(7)]
        [InlineData(-42)]
        [InlineData(1234567)]
        [InlineData(int.MaxValue)]
        [InlineData(-int.MaxValue)]
        public void IntegersAsDotNetWritesThem(int value)
        {
            Assert.Equal(value.ToString(CultureInfo.InvariantCulture), Of(sb => sb.Concat(value)));
        }

        [Theory]
        [InlineData(5u, 3u, '0', "005")]
        [InlineData(123u, 2u, '0', "123")]      // never truncated
        [InlineData(9u, 4u, ' ', "   9")]
        public void UnsignedPadding(uint value, uint pad, char padChar, string expected)
        {
            Assert.Equal(expected, Of(sb => sb.Concat(value, pad, padChar, 10)));
        }

        [Theory]
        [InlineData(255u, 16u, "FF")]
        [InlineData(5u, 2u, "101")]
        [InlineData(0u, 16u, "0")]
        public void UnsignedInAnyBase(uint value, uint numberBase, string expected)
        {
            Assert.Equal(expected, Of(sb => sb.Concat(value, 0, '0', numberBase)));
        }

        [Fact]
        public void NegativeIntegerPaddedKeepsItsSign()
        {
            Assert.Equal("-007", Of(sb => sb.Concat(-7, 3u, '0', 10u)));
        }

        /// <summary>
        /// The short integer overloads are the full one with its defaults: base ten, padded with '0' unless another pad
        /// character is given, never truncated, appended after what the builder already holds.
        /// </summary>
        [Fact]
        public void ShortIntegerOverloadsUseBaseTenAndZeroPadding()
        {
            Assert.Equal("4294967295", Of(sb => sb.Concat(uint.MaxValue)));
            Assert.Equal("0", Of(sb => sb.Concat(0u)));
            Assert.Equal("0042", Of(sb => sb.Concat(42u, 4u)));
            Assert.Equal("12345", Of(sb => sb.Concat(12345u, 2u)));
            Assert.Equal("**42", Of(sb => sb.Concat(42u, 4u, '*')));

            Assert.Equal("-0042", Of(sb => sb.Concat(-42, 4u)));
            Assert.Equal("0042", Of(sb => sb.Concat(42, 4u)));
            Assert.Equal("-2147483648", Of(sb => sb.Concat(int.MinValue, 0u)));
            Assert.Equal("-  7", Of(sb => sb.Concat(-7, 3u, ' ')));
            Assert.Equal("   7", Of(sb => sb.Concat(7, 4u, ' ')));

            // each returns the builder it was given, so calls chain onto the text already there
            var builder = new StringBuilder("t=");
            Assert.Same(builder, builder.Concat(5u).Concat(6u, 2u).Concat(7u, 2u, ' ').Concat(-8, 2u).Concat(9, 2u, ' '));
            Assert.Equal("t=506 7-08 9", builder.ToString());
        }

        [Theory]
        [InlineData(12.25f, 2u, "12.25")]
        [InlineData(-3.5f, 1u, "-3.5")]
        [InlineData(-0.75f, 2u, "-0.75")]       // the sign of a value between -1 and 0
        [InlineData(0f, 3u, "0.000")]
        [InlineData(1013.2f, 1u, "1013.2")]
        [InlineData(2.4f, 0u, "2")]
        [InlineData(2.6f, 0u, "3")]
        [InlineData(-2.6f, 0u, "-3")]
        public void FloatsWithDecimalPlaces(float value, uint places, string expected)
        {
            Assert.Equal(expected, Of(sb => sb.Concat(value, places)));
        }

        [Fact]
        public void FloatsAgreeWithDotNetAwayFromRoundingMidpoints()
        {
            var rng = new Random(1234);
            for (int i = 0; i < 2000; i++)
            {
                float v = (float)((rng.NextDouble() - 0.5) * Math.Pow(10, rng.Next(-2, 7)));
                uint places = (uint)rng.Next(1, 4);
                double scaled = Math.Abs(v) * Math.Pow(10, places);
                if (Math.Abs(scaled - Math.Floor(scaled) - 0.5) < 0.01)
                    continue;   // within a hair of a midpoint: banker's and .NET's rounding may differ there
                Assert.Equal(((double)v).ToString("F" + places, CultureInfo.InvariantCulture), Of(sb => sb.Concat(v, places)));
            }
        }

        [Fact]
        public void FloatDefaultsToFiveDecimalPlaces()
        {
            Assert.Equal("1.50000", Of(sb => sb.Concat(1.5f)));
        }

        [Fact]
        public void FloatPaddingPadsTheIntegerPart()
        {
            Assert.Equal("007.50", Of(sb => sb.Concat(7.5f, 2u, 3u)));
        }

        [Fact]
        public void NonNumericFloats()
        {
            Assert.Equal("NaN", Of(sb => sb.Concat(float.NaN, 2u)));
            Assert.Equal("Infinity", Of(sb => sb.Concat(float.PositiveInfinity, 2u)));
            Assert.Equal("-Infinity", Of(sb => sb.Concat(float.NegativeInfinity, 2u)));
        }

        [Fact] // regression: past the int range the rounded value overflowed its int cast
        public void FloatsBeyondTheIntRange()
        {
            Assert.Equal("3000000000.0", Of(sb => sb.Concat(3e9f, 1u)));
            Assert.Equal("-3000000000", Of(sb => sb.Concat(-3e9f, 0u)));
            Assert.Equal("2147483520", Of(sb => sb.Concat(2147483520f, 0u)));
        }

        [Fact]
        public void FormatterReplacesItsPlaceholders()
        {
            var f = new StringFormatter("Mach <<<mach>>> at <<<alt>>> km");
            Assert.Equal("Mach 2.5 at 12 km", f.ToString(new Dictionary<string, object> { ["mach"] = 2.5, ["alt"] = 12 }));
        }

        [Fact]
        public void FormatterLeavesUnknownPlaceholdersAndText()
        {
            var f = new StringFormatter("<<<a>>> and <<<b>>>!");
            Assert.Equal("1 and <<<b>>>!", f.ToString(new Dictionary<string, object> { ["a"] = 1 }));
            Assert.Equal("plain", new StringFormatter("plain").ToString(new Dictionary<string, object>()));
            Assert.Equal("open <<<x", new StringFormatter("open <<<x").ToString(new Dictionary<string, object> { ["x"] = 1 }));
        }

        [Fact]
        public void FormatterTakesANewFormatAsAConfigValue()
        {
            var f = new StringFormatter("<<<v>>>");
            f.Set((object)"[<<<v>>>]");
            Assert.Equal("[<<<v>>>]", f.Get());
            Assert.Equal("[x]", f.ToString(new Dictionary<string, object> { ["v"] = "x" }));
            f.Set((object)42);   // not a string: ignored
            Assert.Equal("[<<<v>>>]", f.ToString());
        }
    }
}
