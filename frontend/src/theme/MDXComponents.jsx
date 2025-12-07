import React from 'react';
import DefaultMDXComponents from '@theme-original/MDXComponents';
import Chatbot from '../components/Chatbot';
import InteractiveQuiz from '../components/InteractiveQuiz';
import InteractiveContent from '../components/InteractiveContent';
import CourseLayout from '../components/CourseLayout';
import InteractiveDiagram from '../components/InteractiveDiagram';
import Interactive3DViewer from '../components/Interactive3DViewer';
import InteractiveSimulation from '../components/InteractiveSimulation';

const MDXComponents = {
  ...DefaultMDXComponents,
  Chatbot,
  InteractiveQuiz,
  InteractiveContent,
  CourseLayout,
  InteractiveDiagram,
  Interactive3DViewer,
  InteractiveSimulation,
};

export default MDXComponents;