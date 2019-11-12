from setuptools import setup

setup(name='systemcontrol',
      version='1.0',
      description='Python package for developing controllers for various control systems',
      URL='https://github.gatech.edu/factslab/systemcontrol',
      author='Rohit Konda',
      author_email='rkonda6@gatech.edu',
      license='None',
      packages=['systemcontrol'],
      install_requires=[
          'numpy', 'matplotlib', 'cxopt'
      ],
      classifiers=[
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.5',
      ],
      zip_safe=False)
