//---------------------------------------------------------------------------- 
//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
// Description: Implementation of CombinedGeometry 
//
// History: 
//      2004/11/11-Michka 
//          Created it
// 
//---------------------------------------------------------------------------

using System;
using MS.Internal;
using System.ComponentModel;
using System.ComponentModel.Design.Serialization;
using System.Diagnostics;
using System.Reflection;
using System.Collections;
using System.Text;
using System.Globalization;
using System.Windows.Media;
using System.Windows;
using System.Windows.Media.Composition;
using System.Text.RegularExpressions;
using System.Windows.Media.Animation;
using System.Windows.Markup;
using System.Runtime.InteropServices;

using SR = MS.Internal.PresentationCore.SR;
using SRID = MS.Internal.PresentationCore.SRID;

namespace PMKS_Silverlight_App
{
    internal enum GeometryCombineMode
    {
                      Union, Intersect, Exclude , Xor
    }
    /// <summary> 
    /// CombinedGeometry
    /// </summary> 
    internal sealed partial class CombinedGeometry : Geometry
    {
        #region Constructors

        /// <summary>
        /// Default constructor 
        /// </summary> 
        internal CombinedGeometry()
        {             
        }

        /// <summary>
        /// Constructor from 2 operands 
        /// </summary>
        /// <param name="geometry1"> 
        /// First geometry to combine 
        /// </param>
        /// <param name="geometry2"> 
        /// Second geometry to combine
        /// </param>
        internal CombinedGeometry(
            Geometry geometry1,
            Geometry geometry2
        )
        {
            Geometry1 = geometry1;
            Geometry2 = geometry2;
        }

        /// <summary>
        /// Constructor from combine mode and 2 operands 
        /// </summary>
        /// <param name="geometryCombineMode"> 
        /// Combine mode - Union, Intersect, Exclude or Xor 
        /// </param>
        /// <param name="geometry1"> 
        /// First geometry to combine
        /// </param>
        /// <param name="geometry2">
        /// Second geometry to combine 
        /// </param>
        internal CombinedGeometry(
            GeometryCombineMode geometryCombineMode,
            Geometry geometry1,
            Geometry geometry2
        )
        {
            this.geometryCombineMode = geometryCombineMode;
            Geometry1 = geometry1;
            Geometry2 = geometry2;
        }

        public GeometryCombineMode geometryCombineMode { get; set; }

        /// <summary>
        /// Constructor from combine mode, 2 operands and a transformation 
        /// </summary>
        /// <param name="geometryCombineMode">
        /// Combine mode - Union, Intersect, Exclude or Xor
        /// </param> 
        /// <param name="geometry1">
        /// First geometry to combine 
        /// </param> 
        /// <param name="geometry2">
        /// Second geometry to combine 
        /// </param>
        /// <param name="transform">
        /// Transformation to apply to the result
        /// </param> 
        internal CombinedGeometry(
            GeometryCombineMode geometryCombineMode,
            Geometry geometry1,
            Geometry geometry2,
            Transform transform)
        {
            GeometryCombineMode = geometryCombineMode;
            Geometry1 = geometry1;
            Geometry2 = geometry2;
            Transform = transform;
        }

        #endregion

        #region Bounds
        /// <summary>
        /// Gets the bounds of this Geometry as an axis-aligned bounding box
        /// </summary> 
        internal override Rect Bounds
        {
            get
            {
                ReadPreamble();

                // GetAsPathGeometry() checks if the geometry is valid
                return GetAsPathGeometry().Bounds;
            }
        }
        #endregion

        #region GetBoundsInternal
        /// <summary> 
        /// Gets the bounds of this Geometry as an axis-aligned bounding box given a Pen and/or Transform
        /// </summary>
        internal override Rect GetBoundsInternal(Pen pen, Matrix matrix, double tolerance, ToleranceType type)
        {
            if (IsObviouslyEmpty())
            {
                return Rect.Empty;
            }

            return GetAsPathGeometry().GetBoundsInternal(pen, matrix, tolerance, type);
        }
        #endregion

        #region Hit Testing
        /// <summary> 
        /// Returns if point is inside the filled geometry. 
        /// </summary>
        internal override bool ContainsInternal(Pen pen, Point hitPoint, double tolerance, ToleranceType type)
        {
            if (pen == null)
            {
                ReadPreamble();

                // Hit the two operands 
                bool hit1 = false;
                bool hit2 = false;

                Transform transform = Transform;
                if (transform != null && !transform.IsIdentity)
                {
                    // Inverse-transform the hit point 
                    Matrix matrix = transform.Value;
                    if (matrix.HasInverse)
                    {
                        matrix.Invert();
                        hitPoint *= matrix;
                    }
                    else
                    {
                        // The matrix will collapse the geometry to nothing, containing nothing 
                        return false;
                    }
                }

                Geometry geometry1 = Geometry1;
                Geometry geometry2 = Geometry2;
                if (geometry1 != null)
                {
                    hit1 = geometry1.ContainsInternal(pen, hitPoint, tolerance, type);
                }
                if (geometry2 != null)
                {
                    hit2 = geometry2.ContainsInternal(pen, hitPoint, tolerance, type);
                }

                // Determine containment according to the theoretical definition
                switch (GeometryCombineMode)
                {
                    case GeometryCombineMode.Union:
                        return hit1 || hit2;

                    case GeometryCombineMode.Intersect:
                        return hit1 && hit2;

                    case GeometryCombineMode.Exclude:
                        return hit1 && !hit2;

                    case GeometryCombineMode.Xor:
                        return hit1 != hit2;
                }

                // We should have returned from one of the cases 
                Debug.Assert(false);
                return false;
            }
            else
            {
                // pen != null 
                return base.ContainsInternal(pen, hitPoint, tolerance, type);
            }
        }

        #endregion

        /// <summary> 
        /// Gets the area of this geometry
        /// </summary> 
        /// <param name="tolerance">The computational error tolerance</param> 
        /// <param name="type">The way the error tolerance will be interpreted - realtive or absolute</param>
        internal override double GetArea(double tolerance, ToleranceType type)
        {
            ReadPreamble();

            // Potential speedup, to be done if proved important:  As the result of a Combine 
            // operation, the result of GetAsPathGeometry() is guaranteed to be organized into
            // flattened well oriented figures.  Its area can therefore be computed much faster 
            // without the heavy machinary of CArea.  This will require writing an internal 
            // CShapeBase::GetRawArea method, and a utility to invoke it.  For now:
            return GetAsPathGeometry().GetArea(tolerance, type);
        }

        #region Internal

        internal override PathFigureCollection GetTransformedFigureCollection(Transform transform)
        {
            return GetAsPathGeometry().GetTransformedFigureCollection(transform);
        }

        /// <summary>
        /// GetPathGeometryData - returns a struct which contains this Geometry represented
        /// as a path geometry's serialized format.
        /// </summary> 
        internal override PathGeometryData GetPathGeometryData()
        {
            if (IsObviouslyEmpty())
            {
                return Geometry.GetEmptyPathGeometryData();
            }

            PathGeometry pathGeometry = GetAsPathGeometry();

            return pathGeometry.GetPathGeometryData();
        }

        internal override PathGeometry GetAsPathGeometry()
        {
            // Get the operands, interpreting null as empty PathGeometry
            Geometry g1 = Geometry1;
            Geometry g2 = Geometry2;
            PathGeometry geometry1 = (g1 == null) ?
                new PathGeometry() :
                g1.GetAsPathGeometry();

            Geometry geometry2 = (g2 == null) ?
                new PathGeometry() :
                g2.GetAsPathGeometry();

            // Combine them and return the result
            return Combine(geometry1, geometry2, GeometryCombineMode, Transform);
        }


        #region Combine
        /// <summary>
        /// Returns the result of a Boolean combination of two Geometry objects. 
        /// </summary>
        /// <param name="geometry1">The first Geometry object</param> 
        /// <param name="geometry2">The second Geometry object</param> 
        /// <param name="mode">The mode in which the objects will be combined</param>
        /// <param name="transform">A transformation to apply to the result, or null</param> 
        /// <param name="tolerance">The computational error tolerance</param>
        /// <param name="type">The way the error tolerance will be interpreted - relative or absolute</param>
        ///<SecurityNote>
        ///     Critical - calls code that perfoms an elevation ( MilUtility_PathGeometryCombine ) 
        ///     TreatAsSafe - the net effect of this function is to return a new PathGeometry given a transform and a combine operation.
        ///                          Considered safe. 
        /// 
        ///                          Although we call code within an unsafe block - managed objects are used to construct the unmanaged data.
        ///                          unsafe code will have to be reviewed 
        ///</SecurityNote>
        internal static PathGeometry InternalCombine(
            Geometry geometry1,
            Geometry geometry2,
            GeometryCombineMode mode,
            Transform transform,
            double tolerance,
            ToleranceType type)
        {
            PathGeometry resultGeometry = null;

            unsafe
            {
                MilMatrix3x2D matrix = CompositionResourceManager.TransformToMilMatrix3x2D(transform);

                PathGeometryData data1 = geometry1.GetPathGeometryData();
                PathGeometryData data2 = geometry2.GetPathGeometryData();

                fixed (byte* pPathData1 = data1.SerializedData)
                {
                    Debug.Assert(pPathData1 != (byte*)0);

                    fixed (byte* pPathData2 = data2.SerializedData)
                    {
                        Debug.Assert(pPathData2 != (byte*)0);

                        FillRule fillRule = FillRule.Nonzero;

                        FigureList list = new FigureList();
                        int hr = UnsafeNativeMethods.MilCoreApi.MilUtility_PathGeometryCombine(
                            &matrix,
                            &data1.Matrix,
                            data1.FillRule,
                            pPathData1,
                            data1.Size,
                            &data2.Matrix,
                            data2.FillRule,
                            pPathData2,
                            data2.Size,
                            tolerance,
                            type == ToleranceType.Relative,
                            new AddFigureToListDelegate(list.AddFigureToList),
                            mode,
                            out fillRule);

                        if (hr == (int)MILErrors.WGXERR_BADNUMBER)
                        {
                            // When we encounter NaNs in the renderer, we absorb the error and draw 
                            // nothing. To be consistent, we return an empty geometry.
                            resultGeometry = new PathGeometry();
                        }
                        else
                        {
                            HRESULT.Check(hr);

                            resultGeometry = new PathGeometry(list.Figures, fillRule, null);
                        }
                    }
                }
            }

            return resultGeometry;
        }
        #endregion Combine


        #endregion

        #region IsEmpty

        /// <summary>
        /// Returns true if this geometry is empty
        /// </summary> 
        internal override bool IsEmpty()
        {
            return GetAsPathGeometry().IsEmpty();
        }

        internal override bool IsObviouslyEmpty()
        {
            // See which operand is obviously empty
            Geometry geometry1 = Geometry1;
            Geometry geometry2 = Geometry2;
            bool empty1 = geometry1 == null || geometry1.IsObviouslyEmpty();
            bool empty2 = geometry2 == null || geometry2.IsObviouslyEmpty();

            // Depending on the operation -- 
            if (GeometryCombineMode == GeometryCombineMode.Intersect)
            {
                return empty1 || empty2;
            }
            else if (GeometryCombineMode == GeometryCombineMode.Exclude)
            {
                return empty1;
            }
            else
            {
                // Union or Xor
                return empty1 && empty2;
            }
        }


        #endregion IsEmpty

        /// <summary>
        /// Returns true if this geometry may have curved segments
        /// </summary>
        internal override bool MayHaveCurves()
        {
            Geometry geometry1 = Geometry1;
            Geometry geometry2 = Geometry2;
            return ((geometry1 != null) && geometry1.MayHaveCurves())
                ||
                   ((geometry2 != null) && geometry2.MayHaveCurves());
        }
    }
}

// File provided for Reference Use Only by Microsoft Corporation (c) 2007.
