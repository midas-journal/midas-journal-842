
/*=========================================================================

  Program:   Intensity based 2D3D registration

=========================================================================*/

#include <iostream>
#include <fstream>

#include "itkImageRegistrationMethod.h"

#include "itkEuler3DTransform.h"

#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"

#include "itkRayCastInterpolateImageFunction.h"

#include "itkRegularStepGradientDescentOptimizer.h"

#include "itkImage.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkCastImageFilter.h"

#include "itkCommand.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//record every parameters into 8 separate files during registration procedure
ofstream simfile ("similarity.txt");  
ofstream alffile ("alfa.txt");        
ofstream pa1file ("parameter1.txt"); 
ofstream pa2file ("parameter2.txt");  
ofstream pa3file ("parameter3.txt"); 
ofstream pa4file ("parameter4.txt");  
ofstream pa5file ("parameter5.txt");  
ofstream pa6file ("parameter6.txt");  



class CommandIterationUpdate : public itk::Command 
{
public:
  typedef  CommandIterationUpdate   Self;
  typedef  itk::Command             Superclass;
  typedef itk::SmartPointer<Self>  Pointer;
  itkNewMacro( Self );

protected:
  CommandIterationUpdate() {};

public:
  typedef itk::RegularStepGradientDescentOptimizer     OptimizerType;
  typedef const OptimizerType                         *OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject & event)
  {
    Execute( (const itk::Object *)caller, event);
  }

  void Execute(const itk::Object *object, const itk::EventObject & event)
  {
    OptimizerPointer optimizer = 
                      dynamic_cast< OptimizerPointer >( object );
    if( typeid( event ) != typeid( itk::IterationEvent ) )
      {
        return;
      }
    std::cout << "Iteration: " << optimizer->GetCurrentIteration() << std::endl;
    std::cout << "Similarity: " << optimizer->GetValue() << std::endl;
    std::cout << "Position: " << (optimizer->GetCurrentPosition()[0])*180/M_PI<<'\\'<<(optimizer->GetCurrentPosition()[1])*180/M_PI<<'\\'<<(optimizer->GetCurrentPosition()[2])*180/M_PI<<'\\'<<(optimizer->GetCurrentPosition()[3])<<'\\'<<(optimizer->GetCurrentPosition()[4])<<'\\'<<(optimizer->GetCurrentPosition()[5])<< std::endl;
    std::cout << "Alfa:"  << optimizer ->GetCurrentStepLength() <<std::endl;

    simfile<< optimizer->GetValue()<<endl;               
    alffile<< optimizer->GetCurrentStepLength()<<endl;         
    pa1file<< (optimizer->GetCurrentPosition()[0])*180/M_PI<<endl;   
    pa2file<< (optimizer->GetCurrentPosition()[1])*180/M_PI<<endl;   
    pa3file<< (optimizer->GetCurrentPosition()[2])*180/M_PI<<endl;   
    pa4file<< optimizer->GetCurrentPosition()[3]<<endl;     
    pa5file<< optimizer->GetCurrentPosition()[4]<<endl;       
    pa6file<< optimizer->GetCurrentPosition()[5]<<endl;       

  }
};

 
int main( int argc, char *argv[] )
{

  bool ok;
  bool debug = false;

 int R_X=*argv[3];
 int R_Y=*argv[4];
 int R_Z=*argv[5];
 int T_X=*argv[6];
 int T_Y=*argv[7];
 int T_Z=*argv[8];

  // Software Guide: BeginLatex
  //
  // rx, ry, rz, tx, ty, tz are the 6 initial parameters,
  // specified by the user 
  //
  // Software Guide: EndLatex
  double rx = R_X;
  double ry = R_Y;
  double rz = R_Z;

  double tx = T_X;
  double ty = T_Y;
  double tz = T_Z;

  // Software Guide: Beginlatex
  //
  // Specify focal length and distance from ray source to 3D center. 
  //
  // Software Guid: EndLatex

  double focalLength = 10000.;
  double D = 9500.;        

 // double maxStepSize = alfa[i];
  double maxStepSize = 1;
  double minStepSize = 0.0001;

  double threshold=0;
 
  // Software Guide : BeginLatex
  //
  // We begin the program proper by defining the 2D and 3D images. The
  // \code{ImageRegistrationMethod} requires that both
  // images have the same dimension so the 2D image is given
  // dimension 3 and the size of the \emph{z} dimension is set to unity.
  //
  // Software Guide : EndLatex
  
  const    unsigned int    Dimension = 3;
  typedef  unsigned int           PixelType;

  typedef itk::Image< PixelType, Dimension > ImageType2D;
  typedef itk::Image< PixelType, Dimension > ImageType3D;
 

  // Software Guide : BeginLatex
  //
  // The following lines define each of the components used in the
  // registration: The transform, optimizer, metric, interpolator and
  // the multi-resolution registration method itself.
  //
  //
  // Software Guide : EndLatex
  
  typedef   float    InternalPixelType;
  typedef itk::Image< InternalPixelType, Dimension > InternalImageType;

  typedef itk::Euler3DTransform< double >  TransformType;

  typedef itk::RegularStepGradientDescentOptimizer       OptimizerType;

  typedef itk::NormalizedMutualInformationHistogramImageToImageMetric< 
                                    InternalImageType, 
                                    InternalImageType >    MetricType;

  typedef itk::RayCastInterpolateImageFunction< 
                                    InternalImageType,
                                    double          >    InterpolatorType;

  
  typedef itk::ImageRegistrationMethod< 
                                    InternalImageType, 
                                    InternalImageType >   RegistrationType;

  MetricType::Pointer         metric        = MetricType::New();
  TransformType::Pointer      transform     = TransformType::New();
  OptimizerType::Pointer      optimizer     = OptimizerType::New();
  InterpolatorType::Pointer   interpolator  = InterpolatorType::New();
  RegistrationType::Pointer   registration  = RegistrationType::New();

  registration->SetMetric(        metric        );
  registration->SetOptimizer(     optimizer     );
  registration->SetTransform(     transform     );
  registration->SetInterpolator(  interpolator  );

  if (debug) 
    {
    //metric->DebugOn();
    transform->DebugOn();   
    //optimizer->DebugOn();   
    //interpolator->DebugOn();
    registration->DebugOn();
    }


  // Software Guide : BeginLatex
  //
  //  The 2-D and 3-D images are read from input arguments, 
  //
  // Software Guide : EndLatex

  typedef itk::ImageFileReader< ImageType2D > ImageReaderType2D;
  typedef itk::ImageFileReader< ImageType3D > ImageReaderType3D;

  ImageReaderType2D::Pointer imageReader2D = ImageReaderType2D::New();
  ImageReaderType3D::Pointer imageReader3D = ImageReaderType3D::New();

  imageReader2D->SetFileName( argv[1] );
  imageReader3D->SetFileName( argv[2] );


  // Software Guide : BeginLatex
  //
  //  but before connecting these images to the registration we need
  //  to cast them to the internal image type using
  //  \doxygen{CastImageFilters}.
  //
  // Software Guide : EndLatex


  typedef itk::CastImageFilter< 
                        ImageType2D, InternalImageType > CastFilterType2D;
  typedef itk::CastImageFilter< 
                        ImageType3D, InternalImageType > CastFilterType3D;

  CastFilterType2D::Pointer caster2D = CastFilterType2D::New();
  CastFilterType3D::Pointer caster3D = CastFilterType3D::New();


  // Software Guide : BeginLatex
  //
  //  The output of the readers is connected as input to the cast
  //  filters. The inputs to the registration method are taken from the
  //  cast filters. 
  //
  // Software Guide : EndLatex

  imageReader2D->Update();
  imageReader3D->Update();

  caster2D->SetInput( imageReader2D->GetOutput() );
  caster3D->SetInput( imageReader3D->GetOutput() );

  caster2D->Update();
  caster3D->Update(); 

  registration->SetFixedImage(  caster2D->GetOutput() );
  registration->SetMovingImage( caster3D->GetOutput() );

  registration->SetFixedImageRegion( 
       caster2D->GetOutput()->GetBufferedRegion() );
   

  // Initialise the transform
  // ~~~~~~~~~~~~~~~~~~~~~~~~

  transform->SetComputeZYX(true);

  // Software Guide : BeginLatex
  //
  // The transform is initialised with the translation [tx,ty,tz] and
  // rotation [rx,ry,rz] 
  //
  // Software Guide : EndLatex

  TransformType::OutputVectorType translation;

  translation[0] = tx;
  translation[1] = ty;
  translation[2] = tz;

  transform->SetTranslation(translation);
  transform->SetRotation(M_PI/180.0*rx, M_PI/180.0*ry, M_PI/180.0*rz);
  //transform->SetRotation(rx,ry,rz);

  // Software Guide : BeginLatex
  //
  // The centre of rotation is set by default to the centre of the 3D
  // volume.
  //
  // Software Guide : EndLatex

  double origin3D[ Dimension ];

  const itk::Vector<double, 3> resolution3D = caster3D->GetOutput()->GetSpacing();

  typedef ImageType3D::RegionType      ImageRegionType3D;
  typedef ImageRegionType3D::SizeType  SizeType3D;

  caster3D->Update(); 

  ImageRegionType3D region3D = caster3D->GetOutput()->GetBufferedRegion();
  SizeType3D        size3D   = region3D.GetSize();

  TransformType::InputPointType center;
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;

  origin3D[0] = center[0] - resolution3D[0]*((double) size3D[0] )/2.; 
  origin3D[1] = center[1] - resolution3D[1]*((double) size3D[1] )/2.; 
  origin3D[2] = center[2] - resolution3D[2]*((double) size3D[2] )/2.; 

  caster3D -> GetOutput() -> SetOrigin(origin3D);

  transform->SetCenter(center);


  // Set the position of the 2D image
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  double origin2D[ Dimension ];

  const itk::Vector<double, 3> resolution2D = caster2D->GetOutput()->GetSpacing();

  typedef ImageType2D::RegionType      ImageRegionType2D;
  typedef ImageRegionType2D::SizeType  SizeType2D;

  caster2D -> Update();

  ImageRegionType2D region2D = caster2D->GetOutput()->GetBufferedRegion();
  SizeType2D        size2D   = region2D.GetSize();

  origin2D[0] = center[0] - resolution2D[0]*((double) size2D[0] )/2.; 
  origin2D[1] = center[1] - resolution2D[1]*((double) size2D[1] )/2.; 
  origin2D[2] = center[2] + focalLength - D; 

  caster2D->GetOutput()->SetOrigin( origin2D );


  InterpolatorType::InputPointType focalPoint;

  focalPoint[0] = center[0];
  focalPoint[1] = center[1];
  focalPoint[2] = center[2] - D;

  interpolator->SetFocalPoint(focalPoint);
  interpolator->SetThreshold(threshold);
  interpolator->SetTransform(transform);

  // Set up the transform and start position
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // Software Guide : BeginLatex
  //
  // The registration start position is intialised using the
  // transformation parameters.
  //
  // Software Guide : EndLatex

  registration->SetInitialTransformParameters( transform->GetParameters() );
 

  // Software Guide : BeginLatex
  //
  // We wish to maximize the gradient difference similarity measure.
  //
  // Software Guide : EndLatex

  optimizer->SetMaximize( true ); 

  // Software Guide : BeginLatex
  //
  // The maximum and minimum step lengths are specified. The default
  // values for these parameters are 0.4 and 0 (degrees or mm, depending
  // on the parameter) respectively.
  // The relaxation factor is specified here, and the default value is 0.5
  //
  // Software Guide : EndLatex

  optimizer->SetRelaxationFactor(0.9);
  optimizer->SetMaximumStepLength( maxStepSize );  
  optimizer->SetMinimumStepLength( minStepSize );

  // Software Guide : BeginLatex
  //
  // The maximum number of iterations is set to 1000.
  //
  // Software Guide : EndLatex

  optimizer->SetNumberOfIterations(300);


  //unsigned int numberofBins = 255;
  //metric->SetNumberOfHistogramBins(numberofBins);

  // Software Guide : BeginLatex
  //
  // The optimizer scales are set here. 
  // 
  //
  // Software Guide : EndLatex

  typedef OptimizerType::ScalesType  OptimizerScalesType;
  OptimizerScalesType optimizerScales(transform->GetNumberOfParameters() );

  optimizerScales[0] = 0.6;   //Sr can be specified by user
  optimizerScales[1] = 0.8;
  optimizerScales[2] = 1.0;
  optimizerScales[3] = 0.0005;  //St can be specified by user
  optimizerScales[4] = 0.002;
  optimizerScales[5] = 0.001;

  optimizer->SetScales( optimizerScales );


  // Create the observers
  // ~~~~~~~~~~~~~~~~~~~~

  CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();

  optimizer->AddObserver( itk::IterationEvent(), observer );


  // Start the registration
  // ~~~~~~~~~~~~~~~~~~~~~~

  try 
    { 
    // Software Guide : BeginLatex
    //
    // and start the registration.
    //
    // Software Guide : EndLatex

    registration->StartRegistration(); 

    } 
  catch( itk::ExceptionObject & err ) 
    { 
      std::cout << "ExceptionObject caught !" << std::endl; 
      std::cout << err << std::endl; 
      return -1;
    } 



  typedef RegistrationType::ParametersType ParametersType;
  ParametersType finalParameters = registration->GetLastTransformParameters();

  const double RotationAlongX = finalParameters[0];
  const double RotationAlongY = finalParameters[1];
  const double RotationAlongZ = finalParameters[2];
  const double TranslationX = finalParameters[3];
  const double TranslationY = finalParameters[4];
  const double TranslationZ = finalParameters[5];


  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();

  const double bestValue = optimizer->GetValue();


  std::cout << "Result = " << std::endl;
  std::cout << " Rotation X = " << RotationAlongX  << std::endl;
  std::cout << " Rotation Y = " << RotationAlongY  << std::endl;
  std::cout << " Rotation Z = " << RotationAlongZ  << std::endl;  

  std::cout << " Translation X = " << TranslationX  << std::endl;
  std::cout << " Translation Y = " << TranslationY  << std::endl;
  std::cout << " Translation Z = " << TranslationZ  << std::endl;
  std::cout << " Iterations    = " << numberOfIterations << std::endl;
  std::cout << " Metric value  = " << bestValue          << std::endl;

 
  simfile.close();
  alffile.close();
  pa1file.close();
  pa2file.close();
  pa3file.close();
  pa4file.close();
  pa5file.close();
  pa6file.close();

  return 0;
}

