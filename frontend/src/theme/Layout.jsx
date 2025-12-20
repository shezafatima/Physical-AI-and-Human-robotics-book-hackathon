// import React, { useState, useEffect } from 'react';
// import OriginalLayout from '@theme-original/Layout';
// import { ThemeProvider } from '../contexts/ThemeContext';
// import Chatbot from '../components/Chatbot';


// const Layout = (props) => {
//   const [showChatbot, setShowChatbot] = useState(false);
//   const [isChatbotOpen, setIsChatbotOpen] = useState(false);

//   // Check if chatbot should be shown based on props
//   useEffect(() => {
//     if (props.showChatbot) {
//       setShowChatbot(true);
//       // Set to open by default when showing on homepage, similar to how it appears when showChatbot is true
//       setIsChatbotOpen(true);
//     }
//   }, [props.showChatbot]);

//   const toggleChatbot = () => {
//     setIsChatbotOpen(!isChatbotOpen);
//   };

//   return (
//     <ThemeProvider>
//       <OriginalLayout {...props}>
//         {/* Add theme toggle in the main layout, positioned to not conflict with navbar */}
//         <div style={{ position: 'fixed', top: '80px', right: '1rem', zIndex: 10000 }}>
//           <ThemeToggle />
//         </div>
//         {props.children}

//         {/* Floating Chatbot - similar to CourseLayout */}
//         {showChatbot && (
  //           <div style={{
    //             position: 'fixed',
    //             bottom: '20px',
//             right: '20px',
//             zIndex: 10000
//           }}>
//             <div style={{
//               display: 'flex',
//               flexDirection: 'column',
//               alignItems: 'flex-end'
//             }}>
//               <button
//                 onClick={toggleChatbot}
//                 style={{
  //                   background: '#2563eb',
  //                   color: 'white',
  //                   border: 'none',
  //                   borderRadius: '50%',
  //                   width: '60px',
  //                   height: '60px',
  //                   fontSize: '24px',
  //                   cursor: 'pointer',
  //                   boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
  //                   marginBottom: '10px'
  //                 }}
  //                 aria-label={isChatbotOpen ? "Close chatbot" : "Open chatbot"}
  //               >
  //                 {isChatbotOpen ? '×' : '💬'}
//               </button>
//               {isChatbotOpen && <Chatbot onClose={toggleChatbot} />}
//             </div>
//           </div>
//         )}
//       </OriginalLayout>
//     </ThemeProvider>
//   );
// };

// export default Layout;
// import React from 'react';
// import OriginalLayout from '@theme-original/Layout';
// import { ThemeProvider } from '../contexts/ThemeContext';
// import Chatbot from '../components/Chatbot';
// import ThemeToggle from '../components/ThemeToggle';

// const Layout = (props) => {
//   return (
//     <ThemeProvider>
//       <OriginalLayout {...props}>
//         {/* Theme toggle, optional */}
//         <div style={{ position: 'fixed', top: '80px', right: '1rem', zIndex: 10000 }}>
//           <ThemeToggle />
//         </div>

//         {/* Main content */}
//         {props.children}

//         {/* Just render your Chatbot directly, no extra buttons or floating logic */}
//         <Chatbot />
//       </OriginalLayout>
//     </ThemeProvider>
//   );
// };

// export default Layout;
                 
import React, { useState } from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ThemeProvider } from '../contexts/ThemeContext';
import Chatbot from '../components/Chatbot';
import ThemeToggle from '../components/ThemeToggle';

import styles from '../components/CourseLayout.module.css';

const Layout = (props) => {
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  const toggleChatbot = () => {
    setIsChatbotOpen((prev) => !prev);
  };

  return (
    <ThemeProvider>
      <OriginalLayout {...props}>
               <div style={{ position: 'fixed', top: '80px', right: '1rem', zIndex: 10000 }}>
         <ThemeToggle />
       </div>
        {props.children}

        {/* GLOBAL CHATBOT – SAME UI AS CourseLayout */}
        <div
          className={`${styles.chatbotContainer} ${
            isChatbotOpen ? styles.chatbotOpen : ''
          }`}
        >
          <button
            className={styles.chatbotToggle}
            onClick={toggleChatbot}
            aria-label={isChatbotOpen ? 'Close chatbot' : 'Open chatbot'}
          >
            {isChatbotOpen ? '×' : '💬'}
          </button>

          {isChatbotOpen && <Chatbot onClose={toggleChatbot} />}
        </div>
      </OriginalLayout>
    </ThemeProvider>
  );
};

export default Layout;
